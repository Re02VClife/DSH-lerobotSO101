"""Joint zero calibration for the SO-101 -- the blocker for everything else.

Why this must come first
------------------------
Raw servo ticks are NOT joint angles.  With no calibration, `Present_Position` = 2063
for the shoulder means nothing: the mapping from tick to joint degree depends on where
that particular arm's joint zero physically is.  Feed raw ticks into FK and you get a
confident, wrong pose -- which then silently corrupts the hand-eye calibration, every
IK target, and every grasp.

How lerobot does it, and what we do
-----------------------------------
The official flow (verified in docs/source/so101.mdx and the feetech tables): move every
joint to the MIDDLE of its range, press enter, then move each joint through its full
range so `range_min`/`range_max` can be recorded.  Mechanically it works by writing
`Homing_Offset` (register 31, 2 bytes) on each servo and restarting it, so that the
middle pose comes back as exactly 2048 ticks.  Servo-side registers involved:
    Phase          (18, 1 byte)   -- sts3215 must have bit 4 cleared for angle mode
    Homing_Offset  (31, 2 bytes)
    Torque_Enable  (40, 1 byte)
    Lock           (55, 1 byte)

This tool implements the same middle-pose step in two modes:

  --mode soft    record the raw tick of each joint at the middle pose into a project
                 calibration file.  Nothing is written to the servos, so it is fully
                 reversible, and the bridge converts ticks -> degrees in software.
                 Use this first.
  --mode homing  additionally write `Homing_Offset` on each servo and restart it, which
                 makes the arm report 2048 at the middle pose.  This is what makes the
                 arm interoperable with lerobot tools and leader/follower teleoperation.
                 Requires the arm to be physically at the middle pose.

Either way, the middle pose is defined from the URDF joint limits, and the tool prints
the exact target angle for each joint so you can position the arm against a printed
protractor or against the leader arm.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.kinematics import ALL_JOINTS, LEROBOT_TO_URDF, SO101Kinematics  # noqa: E402
from bridge.servo_bus import (  # noqa: E402
    ADDR_PRESENT_POSITION,
    JOINT_IDS,
    ServoBus,
    TICKS_PER_TURN,
)

ADDR_PHASE = 18
ADDR_HOMING_OFFSET = 31
ADDR_LOCK = 55
ADDR_TORQUE_ENABLE = 40


def middle_pose_deg(kin: SO101Kinematics) -> dict[str, float]:
    """Middle of each joint's range, in lerobot joint names.  This is the calibration pose."""
    out: dict[str, float] = {}
    for lerobot_name, urdf_name in LEROBOT_TO_URDF.items():
        lo, hi = kin.limits_deg[urdf_name]
        out[lerobot_name] = (lo + hi) / 2.0
    return out


def read_raw_ticks(bus: ServoBus) -> dict[str, int]:
    states = bus.read_all()
    return {name: st.ticks for name, st in states.items()}


def build_calibration(
    mid_ticks: dict[str, int],
    kin: SO101Kinematics,
    *,
    direction_sign: dict[str, int] | None = None,
    range_ticks: dict[str, tuple[int, int]] | None = None,
) -> dict:
    """Assemble the project calibration file from the recorded middle-pose ticks."""
    sign = direction_sign or {k: 1 for k in JOINT_IDS}
    calib: dict = {
        "format": "ds101-so101-calibration",
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "port": None,
        "ticks_per_turn": TICKS_PER_TURN,
        "middle_ticks": {k: int(v) for k, v in mid_ticks.items()},
        "direction_sign": {k: int(v) for k, v in sign.items()},
        "joint_limits_deg": {k: list(kin.limits_deg[v]) for k, v in LEROBOT_TO_URDF.items()},
        "middle_pose_deg": middle_pose_deg(kin),
        "notes": (
            "middle_ticks[j] is the raw Present_Position of joint j when the arm is at the "
            "middle of its range. Convert with: deg = mid_deg + (tick - middle_ticks) * "
            "360/4096 * sign.  Valid only while the servo's own Homing_Offset stays unchanged."
        ),
    }
    if range_ticks:
        calib["range_ticks"] = {k: [int(a), int(b)] for k, (a, b) in range_ticks.items()}
    return calib


def ticks_to_deg(ticks: dict[str, int], calib: dict, kin: SO101Kinematics) -> dict[str, float]:
    """Convert raw ticks to lerobot joint degrees using a soft calibration."""
    out: dict[str, float] = {}
    for lname, mid_tick in calib["middle_ticks"].items():
        urdf_name = LEROBOT_TO_URDF[lname]
        sign = calib["direction_sign"].get(lname, 1)
        delta = ticks[lname] - int(mid_tick)
        out[lname] = calib["middle_pose_deg"][lname] + delta * 360.0 / TICKS_PER_TURN * sign
    return out


def deg_to_ticks(deg: dict[str, float], calib: dict) -> dict[str, int]:
    """Inverse of ticks_to_deg: joint degrees -> raw servo ticks."""
    out: dict[str, int] = {}
    for lname, value in deg.items():
        mid_tick = int(calib["middle_ticks"][lname])
        sign = calib["direction_sign"].get(lname, 1)
        delta = (value - calib["middle_pose_deg"][lname]) / sign
        out[lname] = int(round(mid_tick + delta * TICKS_PER_TURN / 360.0))
    return out


def write_homing_offsets(bus: ServoBus, mid_ticks: dict[str, int]) -> dict[str, int]:
    """Make each servo report 2048 at the current (middle) pose, then restart it.

    Sequence per servo, matching the lerobot behaviour:
      1. torque off (a servo fighting the write will snap when it restarts)
      2. clear bit 4 of Phase so the arm uses angle feedback mode (sts3215 only)
      3. Homing_Offset = (2048 - present_position) mod 4096, as a signed 11-bit value
      4. restart the servo by toggling Torque_Enable off/on, then re-read
    """
    written: dict[str, int] = {}
    for name, sid in JOINT_IDS.items():
        cur = bus._read2(sid, ADDR_PRESENT_POSITION)
        offset = ((2048 - cur + 2048) % 4096) - 2048     # signed
        phase = bus._read1(sid, ADDR_PHASE)
        if phase & 0x10:
            bus._write1(sid, ADDR_PHASE, phase & ~0x10)
        bus._write1(sid, ADDR_TORQUE_ENABLE, 0)
        bus._write2(sid, ADDR_HOMING_OFFSET, offset % 4096 if offset >= 0 else offset + 4096)
        bus._write1(sid, ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.05)
        bus._write1(sid, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.05)
        after = bus._read2(sid, ADDR_PRESENT_POSITION)
        written[name] = after
        print(f"  {name:<15} id={sid} present_before={cur:<5} homing_offset={offset:<6} "
              f"present_after={after}")
    return written


def main() -> None:
    ap = argparse.ArgumentParser(description="SO-101 joint zero calibration")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--mode", choices=["soft", "homing"], default="soft")
    ap.add_argument("--out", default="calib/out/so101_calibration.json")
    ap.add_argument("--yes", action="store_true", help="skip the interactive confirmation")
    args = ap.parse_args()

    kin = SO101Kinematics()
    mid = middle_pose_deg(kin)

    print("=" * 78)
    print("SO-101 joint zero calibration")
    print("=" * 78)
    print("\nSTEP 1 -- move the arm BY HAND to the middle of every joint's range.")
    print("Torque stays OFF, so the arm moves freely. Target angles (lerobot convention):\n")
    print(f"  {'joint':<15}{'middle angle':>14}   {'URDF range':>20}")
    for name, value in mid.items():
        lo, hi = kin.limits_deg[LEROBOT_TO_URDF[name]]
        print(f"  {name:<15}{value:>11.1f} deg   {lo:>8.1f} .. {hi:<8.1f}")
    print("\n  Practically: the SO-101 middle pose is roughly")
    print("    - shoulder_pan pointing halfway between its two extremes,")
    print("    - shoulder_lift, elbow_flex, wrist_flex all at their geometric midpoints,")
    print("    - gripper half open.")
    print("  If you have the leader arm, put BOTH arms in the same pose; that is what makes")
    print("  teleoperation work later.")

    with ServoBus(args.port, allow_motion=False) as bus:
        found = bus.scan()
        if sorted(found) != sorted(JOINT_IDS.values()):
            print(f"\nABORT: expected servo ids {sorted(JOINT_IDS.values())} on {args.port}, found {sorted(found)}")
            raise SystemExit(2)
        print(f"\nbus {args.port}: ids {sorted(found)} online, torque is off (safe to move by hand)")

        if not args.yes:
            input("\nPress ENTER once the arm is holding the middle pose... ")

        mid_ticks = read_raw_ticks(bus)
        print("\nrecorded middle-pose ticks:")
        for name, tick in mid_ticks.items():
            print(f"  {name:<15} {tick}")

        if args.mode == "homing":
            print("\nSTEP 2 -- writing Homing_Offset to each servo and restarting it.")
            print("This makes the middle pose read back as exactly 2048 ticks.")
            if not args.yes:
                input("Press ENTER to write (this modifies servo EEPROM)... ")
            before = dict(mid_ticks)
            after = write_homing_offsets(bus, mid_ticks)
            bad = [n for n in after if abs(after[n] - 2048) > 6]
            if bad:
                print(f"\nWARNING: these joints did not come back at 2048: {bad}")
                print("Check for a mechanical obstruction or a servo that refused the write.")
            calib = build_calibration({n: 2048 for n in JOINT_IDS}, kin)
            calib["homing_written"] = True
            calib["present_before_homing"] = before
        else:
            calib = build_calibration(mid_ticks, kin)
            calib["homing_written"] = False

        calib["port"] = args.port
        calib["servo_models"] = {str(k): int(v) for k, v in found.items()}

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(calib, indent=2), encoding="utf-8")
    print(f"\nwritten: {out}")
    print("\nNEXT: verify the calibration before trusting it.")
    print("  1. python bridge/servo_bus.py            # re-read ticks")
    print("  2. move the arm to two poses and check that FK + a tape measure agree")
    print("     to within a few mm over ~200 mm of travel.")
    print("  A wrong direction_sign flips one joint; catch it now, not during a grasp.")


if __name__ == "__main__":
    main()
