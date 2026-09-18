"""Verify the scene simulator: does a grasp actually move a block?

Before wiring this into the bridge, prove the three behaviours the pick-and-place
verification depends on, each in isolation:

  1. closing the jaws on a block marks it carried and stalls the jaws;
  2. a carried block follows the tool tip;
  3. opening the jaws drops it, and *perception* then reports it at its new position.

Step 3 is the point of the whole module. If the scene did not change, "verify the
placement" would be an assertion about the arm's intentions rather than the world.

Run:  .venv\\Scripts\\python.exe bridge\\sim_scene_test.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.driver import ArmDriver  # noqa: E402
from bridge.fake_camera import SyntheticOverheadCamera  # noqa: E402
from bridge.kinematics import SO101Kinematics  # noqa: E402
from bridge.servo_bus import SafetyConfig  # noqa: E402
from bridge.sim import SimServoBus, sim_calibration  # noqa: E402
from bridge.sim_scene import SimScene, default_objects  # noqa: E402
from calib.perception import find_objects, footprint_centre_base_mm  # noqa: E402

TABLE_Z_MM = -115.0
FAST = SafetyConfig(max_step_deg=180.0, max_speed_ticks_s=1900)

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(' -> ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


def main() -> None:
    kin = SO101Kinematics()
    table_z_mm = TABLE_Z_MM
    bus = SimServoBus(allow_motion=True, safety=FAST)
    scene = SimScene(bus, kin, table_z_mm=table_z_mm, objects=default_objects())
    driver = ArmDriver("SIM", allow_motion=True, safety=FAST,
                       calibration=sim_calibration(kin), bus=bus)
    driver.connect()
    camera = SyntheticOverheadCamera(table_z_mm=table_z_mm, blocks=scene.block_dicts())

    def settle():
        """Let the world catch up with the arm.

        Physics now rides the bus's own integration tick (see SimScene), so this is only a
        register read -- every read advances the plant.  It survives as a named step because
        the test reads better saying where it expects the world to have caught up, and
        because a future edit that moves physics back into the caller would have to break
        this line rather than silently stop stepping the world.
        """
        bus.read_all()

    def perceive():
        settle()
        camera.blocks = scene.block_dicts()
        color, depth = camera.render()
        objs, info = find_objects(color, depth, camera.color_intr, T_base_cam=camera.T_base_cam)
        return objs, info

    target = scene.objects[0]
    # Snapshot the truth BEFORE anything moves. `target` is the live object and its
    # centre_mm is mutated as it is carried, so comparing a placement against
    # `target.centre_mm` silently compares against the destination and always "passes".
    source_mm = target.centre_mm.copy()
    print(f"target: {target.name} at {source_mm} size {target.size_mm} h={target.height_mm}")

    print("\n0. the untouched scene is perceived")
    objs, info = perceive()
    check("table plane found", info["table_inlier_fraction"] > 0.9,
          f"{info['table_inlier_fraction']:.3f}")
    check("all three blocks visible", len(objs) == 3, f"{len(objs)} found")
    found = min(objs, key=lambda o: np.linalg.norm(
        np.array(o.centroid_base[:2]) * 1000.0 - target.centre_mm))
    grip = footprint_centre_base_mm(found, camera.eye_mm, table_z_mm)
    err = float(np.linalg.norm(grip[:2] - target.centre_mm))
    check("target located within 15 mm of truth", err < 15.0, f"{err:.1f} mm")
    print(f"      perceived grip point: {[round(float(v), 1) for v in grip]}")

    print("\n1. descending and closing catches the block")
    grasp_z = table_z_mm + target.height_mm + 98.0 + 4.0        # wrist pose for a top grasp
    pre = driver.goto_tool([grip[0], grip[1], grasp_z + 60.0])
    check("approach move reached", pre.get("ok") is True, f"status={pre.get('status')}")
    down = driver.goto_tool([grip[0], grip[1], grasp_z])
    check("descent reached", down.get("ok") is True, f"status={down.get('status')}")
    closed = driver.set_gripper(0.0)
    settle()
    check("closing the jaws reports a possible grasp", closed.get("possible_grasp") is True,
          f"possible_grasp={closed.get('possible_grasp')} measured={closed.get('measured_deg')}")
    check("the scene marks the block as carried", scene.held_object() is target,
          f"held={scene.held_object().name if scene.held_object() else None}")

    print("\n2. the carried block follows the arm")
    lift = driver.goto_tool([grip[0], grip[1], grasp_z + 100.0])
    check("lift reached", lift.get("ok") is True, f"status={lift.get('status')}")
    settle()
    tip = scene.tool_tip_mm()
    follow_err = float(np.linalg.norm(target.centre_mm - np.array([tip[0], tip[1]])))
    check("carried block tracks the tool tip", follow_err < 25.0, f"{follow_err:.1f} mm")

    print("\n3. it is still missing from perception while held")
    objs_held, _ = perceive()
    names_seen = len(objs_held)
    check("only the other two blocks are perceived", names_seen == 2, f"{names_seen} found")

    print("\n4. traversing and releasing puts it down at the new place")
    dest = np.array([330.0, -150.0])
    move = driver.goto_tool([dest[0], dest[1], grasp_z + 100.0])
    check("traverse reached", move.get("ok") is True, f"status={move.get('status')}")
    lower = driver.goto_tool([dest[0], dest[1], grasp_z])
    check("lower reached", lower.get("ok") is True, f"status={lower.get('status')}")
    opened = driver.set_gripper(100.0)
    settle()
    check("the scene considers it released", scene.held_object() is None)
    placed = scene.objects[0]
    place_err = float(np.linalg.norm(placed.centre_mm - dest))
    check("the block landed at the destination", place_err < 25.0,
          f"{place_err:.1f} mm from {dest.tolist()}")

    print("\n5. VERIFICATION: perception now reports it at the new place")
    retract = driver.goto_tool([dest[0], dest[1], grasp_z + 140.0])
    objs_after, _ = perceive()
    check("all three blocks visible again", len(objs_after) == 3, f"{len(objs_after)} found")
    nearest = min(objs_after, key=lambda o: np.linalg.norm(
        np.array(o.centroid_base[:2]) * 1000.0 - dest))
    seen_grip = footprint_centre_base_mm(nearest, camera.eye_mm, table_z_mm)
    residual = float(np.linalg.norm(seen_grip[:2] - dest))
    check("the object is confirmed at the destination within 20 mm", residual < 20.0,
          f"{residual:.1f} mm (grip point {[round(float(v), 1) for v in seen_grip]})")
    away = float(np.linalg.norm(seen_grip[:2] - source_mm))
    check("and is no longer at the source", away > 100.0, f"{away:.1f} mm from the source")

    driver.close()

    print()
    print("scene events:", scene.events)
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s): {FAILURES}")
        raise SystemExit(1)
    print("SCENE SIMULATOR VERIFIED — a pick-and-place physically changes what perception sees, "
          "so the verification step tests the world rather than the arm's intentions.")


if __name__ == "__main__":
    main()
