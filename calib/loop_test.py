"""Closed-loop test: drive the whole pipeline with no hardware and no human.

This is the test that answers "does the thing actually work end to end".  It stands in
for a bench session by replacing only the two things a bench provides -- photons and
metal -- and keeping every line of production code on the path:

    synthetic RGB-D ------> calib.perception.find_objects
                        ------> target selection + reachability
                        ------> calib.perception.tool_target_for_grasp
                        ------> bridge.kinematics IK
                        ------> bridge.driver rate limiting + joint envelope
                        ------> bridge.sim.SimServoBus (integrating plant)
                        ------> FK read-back, compared with the requested target

Everything except the plant is the same code the agent drives through the bridge, so a
failure here is a real failure, and a pass here means the only remaining risk is
mechanical (repeatability, backlash, table level) rather than logical.

What it verifies
----------------
  1. perception recovers both blocks: position, size and height
  2. the reachable block is grasped and the arm arrives inside tolerance
  3. the unreachable block is REFUSED by IK rather than half-attempted
  4. the joint-envelope guard refuses an out-of-range target
  5. the rate limiter forces multi-command motion and the driver reports it honestly
  6. the gripper reports a different result closing on an object than closing on air

Run:  .venv\\Scripts\\python.exe calib\\loop_test.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.cameras import Intrinsics  # noqa: E402
from bridge.driver import ArmDriver  # noqa: E402
from bridge.kinematics import LEROBOT_TO_URDF, SO101Kinematics  # noqa: E402
from bridge.servo_bus import SafetyConfig  # noqa: E402
from bridge.sim import SimServoBus, sim_calibration  # noqa: E402
from calib.perception import (  # noqa: E402
    find_objects,
    footprint_centre_base_mm,
    nearest_object,
    object_top_base_mm,
    tool_target_for_grasp,
)
from calib.selftest import render_workspace  # noqa: E402

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(' -> ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


def main() -> None:
    # ---------------------------------------------------------------- the bench
    TABLE_Z_MM = -127.4
    intr = Intrinsics(width=640, height=480, fx=603.6943359375, fy=602.4212646484375,
                      ppx=316.4139099121094, ppy=234.92718505859375,
                      model="inverse_brown_conrady", coeffs=[0.0] * 5)
    from calib.extrinsics import look_at

    eye = np.array([0.400, 0.0, (610.0 + TABLE_Z_MM) / 1000.0])
    fwd = np.array([-np.sin(np.radians(16.0)), 0.0, -np.cos(np.radians(16.0))])
    up_hint = np.array([0.0, -1.0, 0.0])
    T_base_cam = look_at(eye, fwd, up_hint)
    T_cam_base = look_at(eye, fwd, up_hint, as_cam_from_base=True)
    kin = SO101Kinematics()

    print("1. perception on a synthetic scene")
    reachable = {"centre_mm": (200.0, -60.0), "size_mm": (40.0, 40.0),
                 "height_mm": 45.0, "bgr": (40, 40, 200)}
    # The second block must sit INSIDE the camera's visible table band or the "two blocks
    # detected" assertion is simply wrong.  This mount sees only about 460 mm of table
    # depth, so the far block is placed at the far end of that band, not at an arbitrary
    # distance.
    near_limit = {"centre_mm": (390.0, 250.0), "size_mm": (50.0, 50.0),
                  "height_mm": 35.0, "bgr": (200, 60, 60)}
    unreachable = {"centre_mm": (700.0, 120.0), "size_mm": (50.0, 50.0),
                   "height_mm": 35.0, "bgr": (60, 200, 60)}
    color, depth = render_workspace(intr, T_cam_base, [reachable, near_limit], TABLE_Z_MM)
    check("synthetic depth has a valid table", float((depth > 0).mean()) > 0.9,
          f"{100 * float((depth > 0).mean()):.1f}% valid pixels")

    objects, info = find_objects(color, depth, intr, T_base_cam=T_base_cam)
    check("table plane found", info["table_inlier_fraction"] > 0.9,
          f"inlier fraction {info['table_inlier_fraction']:.3f}")
    print(f"      detected {len(objects)} object(s):")
    for o in objects:
        c = o.centroid_base
        print(f"        id={o.id} base=({c[0] * 1000:.0f}, {c[1] * 1000:.0f}) mm "
              f"h={o.height_mm:.1f} mm bgr={o.bgr}")
    check("both visible blocks detected", len(objects) == 2, f"{len(objects)} found")

    if len(objects) == 2:
        # match detections to truth by proximity, independent of detection order
        for blk in (reachable, near_limit):
            tx, ty = blk["centre_mm"]
            match = nearest_object(objects, (tx, ty), max_distance_mm=40.0)
            ok = match is not None
            check(f"block at ({tx:.0f}, {ty:.0f}) located within 40 mm", ok,
                  "" if not ok else
                  f"detected at ({match.centroid_base[0] * 1000:.0f}, "
                  f"{match.centroid_base[1] * 1000:.0f})")
            if ok:
                check(f"block at ({tx:.0f}, {ty:.0f}) height within 8 mm",
                      abs(match.height_mm - blk["height_mm"]) < 8.0,
                      f"{match.height_mm:.1f} vs {blk['height_mm']}")

    print("\n2. grasp planning and reachability")
    target_block = nearest_object(objects, reachable["centre_mm"][:2]) if objects else None
    check("target block selected", target_block is not None)
    if target_block is None:
        print("\nCannot continue without a detected target.")
        raise SystemExit(1)

    top_base = object_top_base_mm(target_block, TABLE_Z_MM)
    ground_base = footprint_centre_base_mm(target_block, eye * 1000.0, TABLE_Z_MM)
    truth_xy = np.array(reachable["centre_mm"])
    raw_err = float(np.linalg.norm(top_base[:2] - truth_xy))
    corr_err = float(np.linalg.norm(ground_base[:2] - truth_xy))
    print(f"      top-face centre (base mm): {[round(v, 1) for v in top_base]}")
    print(f"      footprint centre corrected: {[round(v, 1) for v in ground_base]}")
    print(f"      plan-view error: raw {raw_err:.1f} mm -> corrected {corr_err:.1f} mm")
    check("the oblique-view correction brings the grip point closer to the true centre",
          corr_err < raw_err and corr_err < 12.0,
          f"{raw_err:.1f} mm -> {corr_err:.1f} mm")

    grasp = tool_target_for_grasp(ground_base, jaw_offset_mm=98.0)
    print(f"      wrist at grasp (mm): {grasp['wrist_at_grasp_mm']}")
    print(f"      pre-grasp wrist (mm): {grasp['pre_grasp_wrist_mm']}")

    seed = {LEROBOT_TO_URDF[k]: v for k, v in sim_calibration(kin)["middle_pose_deg"].items()}
    wrist_target = np.array(grasp["wrist_at_grasp_mm"]) / 1000.0
    ik = kin.ik(wrist_target, seed)
    print(f"      IK: ok={ik['ok']} residual={ik['residual_mm']} mm "
          f"iters={ik['iters']} reason={ik['reason']}")
    check("reachable block has a valid IK solution", ik["ok"], ik["reason"])

    far_top = np.array([unreachable["centre_mm"][0], unreachable["centre_mm"][1],
                        TABLE_Z_MM + unreachable["height_mm"]])
    far_grasp = tool_target_for_grasp(far_top, jaw_offset_mm=98.0)
    far_ik = kin.ik(np.array(far_grasp["wrist_at_grasp_mm"]) / 1000.0, seed)
    check("far block is REFUSED by IK rather than half-attempted",
          not far_ik["ok"], f"residual {far_ik['residual_mm']} mm, {far_ik['reason']}")

    print("\n3. motion through the real driver on a simulated plant")
    bus = SimServoBus(allow_motion=True, safety=SafetyConfig())
    board = SimServoBus(allow_motion=True, safety=SafetyConfig())
    driver = ArmDriver("SIM", allow_motion=True, safety=SafetyConfig(),
                       calibration=sim_calibration(kin), bus=bus)
    driver.connect()

    home = driver.go_home()
    check("homing motion reaches the middle pose", home["ok"] and home["status"] == "reached",
          f"status={home['status']} commands={home['commands_issued']}")
    check("homing took multiple rate-limited commands (the limiter is active)",
          home["commands_issued"] > 1, f"{home['commands_issued']} commands")

    # Cartesian move to the pre-grasp pose, then descend to the grasp pose
    pre = driver.goto_tool(grasp["pre_grasp_wrist_mm"])
    check("pre-grasp move reached", pre.get("ok") is True,
          f"status={pre.get('status')} reason={pre.get('reason')} ik={pre.get('ik')}")
    descend = driver.goto_tool(grasp["wrist_at_grasp_mm"])
    check("grasp descent reached", descend.get("ok") is True,
          f"status={descend.get('status')} reason={descend.get('reason')}")

    arrived = np.array(driver.state()["tool_mm"]) / 1000.0
    err_mm = float(np.linalg.norm(arrived - wrist_target) * 1000)
    check("wrist arrives within 3 mm of the grasp target", err_mm < 3.0,
          f"{err_mm:.2f} mm (requested {grasp['wrist_at_grasp_mm']}, "
          f"arrived {[round(v * 1000, 1) for v in arrived]})")

    print("\n4. the joint envelope guard")
    lo, hi = kin.limits_deg["Elbow"]
    out_of_range = driver.goto_joints({"elbow_flex": hi + 40.0}, max_commands=2)
    check("an out-of-envelope target is clamped and reported as such",
          (not out_of_range["ok"]) and out_of_range["clipped"],
          f"status={out_of_range['status']} clipped={out_of_range['clipped']}")

    print("\n5. the gripper distinguishes air from an object")
    air = driver.set_gripper(0.0)
    air_deg = driver.state()["joint_deg"]["gripper"]
    bus.place_object_in_gripper(2150)
    on_object = driver.set_gripper(0.0)
    obj_deg = driver.state()["joint_deg"]["gripper"]
    print(f"      closing on air   -> gripper {air_deg:.1f} deg, ok={air['ok']}")
    print(f"      closing on object-> gripper {obj_deg:.1f} deg, ok={on_object['ok']}")
    check("closing on an object stalls earlier than closing on air",
          obj_deg < air_deg - 3.0, f"{obj_deg:.1f} vs {air_deg:.1f} deg")

    driver.close()

    print()
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s): {FAILURES}")
        raise SystemExit(1)
    print("CLOSED LOOP VERIFIED - perception, planning, IK, motion and grasp feedback all "
          "agree on a simulated plant.")


if __name__ == "__main__":
    main()
