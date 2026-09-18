"""Turn a recorded joint sweep into calibration: middle ticks, real limits, and a home pose.

What each written field is, and why it is that
----------------------------------------------
`middle_ticks` -- the middle of the OBSERVED travel of each joint, in raw ticks. This is exactly
    the quantity `arm_zero.py --mode soft` writes when you place the arm at the middle pose by
    eye; measuring it from a sweep is the same number without the eyeballing.

`joint_limits_deg` -- the observed travel converted to degrees through the new middle. The URDF
    limits are DESIGN values. On this bench the observed travel is wider for most joints
    (gripper +36 deg) and narrower for one (shoulder_pan -52 deg), and a limit that claims a
    joint can travel further than it physically can is not a safety feature.

`home_ticks` / `home_deg` -- the pose the operator asked to be able to return to. Recorded as
    TICKS, because ticks are what the servos actually report; the degrees are derived for human
    reading only and are exactly as good as the calibration they went through.

This is the `soft` half of calibration: nothing is written to servo EEPROM, the previous file
is kept beside the new one, and the whole thing is reversible by copying it back.

Run:  .venv\\Scripts\\python.exe calib\\apply_range.py --dry-run
      .venv\\Scripts\\python.exe calib\\apply_range.py --write
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.servo_bus import JOINT_IDS, TICKS_PER_TURN  # noqa: E402

JOINTS = list(JOINT_IDS.keys())


def ticks_to_deg(tick: float, middle_tick: float, middle_deg: float, sign: int) -> float:
    return middle_deg + (tick - middle_tick) * (360.0 / TICKS_PER_TURN) * sign


def main() -> None:
    ap = argparse.ArgumentParser(description="Fold a joint sweep into the calibration file.")
    ap.add_argument("--range", default="calib/out/joint_range.json")
    ap.add_argument("--calib", default="calib/out/so101_calibration.json")
    ap.add_argument("--base", default="http://127.0.0.1:8731")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true", help="actually write the calibration")
    mode.add_argument("--dry-run", action="store_true", help="preview only (this is the default)")
    args = ap.parse_args()

    sweep = json.loads(Path(args.range).read_text(encoding="utf-8"))
    calib = json.loads(Path(args.calib).read_text(encoding="utf-8"))

    with urllib.request.urlopen(args.base + "/state", timeout=5) as response:
        live = json.loads(response.read().decode("utf-8"))["state"]
    now_ticks = live["raw_ticks"]

    if sweep.get("samples", 0) < 50:
        print(f"REFUSING: the sweep holds only {sweep.get('samples')} samples.")
        print("A range measured from a handful of readings is not a range.")
        raise SystemExit(2)

    old_middle = dict(calib.get("middle_ticks", {}))
    old_limits = {k: list(v) for k, v in calib.get("joint_limits_deg", {}).items()}

    new_middle: dict[str, int] = {}
    new_limits: dict[str, list[float]] = {}
    home_ticks: dict[str, int] = {}
    home_deg: dict[str, float] = {}

    print(f"{'joint':<16}{'old mid':>9}{'new mid':>9}{'delta':>8}"
          f"{'obs span':>10}{'deg span':>10}{'URDF span':>11}")
    print("-" * 73)
    for name in JOINTS:
        entry = sweep["range"][name]
        if entry["min"] is None or entry["max"] is None:
            print(f"{name:<16} no data -- refusing to guess")
            raise SystemExit(2)
        middle = entry["middle"]
        new_middle[name] = middle
        pose = calib["middle_pose_deg"][name]
        sign = int(calib.get("direction_sign", {}).get(name, 1))
        lo = ticks_to_deg(entry["min"], middle, pose, sign)
        hi = ticks_to_deg(entry["max"], middle, pose, sign)
        new_limits[name] = [round(min(lo, hi), 6), round(max(lo, hi), 6)]
        home_ticks[name] = int(now_ticks[name])
        home_deg[name] = round(ticks_to_deg(now_ticks[name], middle, pose, sign), 2)

        urdf_span = (old_limits.get(name) or [0, 0])
        urdf_span = round(urdf_span[1] - urdf_span[0], 1) if len(urdf_span) == 2 else 0.0
        print(f"{name:<16}{old_middle.get(name, 0):>9}{middle:>9}"
              f"{middle - old_middle.get(name, 0):>+8}{entry['span']:>10}"
              f"{entry['span'] * 360.0 / TICKS_PER_TURN:>10.1f}{urdf_span:>11.1f}")

    print(f"\nhome pose (raw ticks, as the servos report them):")
    for name in JOINTS:
        print(f"  {name:<16}{home_ticks[name]:>6}    ({home_deg[name]:>7.2f} deg by this calibration)")

    calib["middle_ticks"] = new_middle
    calib["joint_limits_deg"] = new_limits
    calib["home_ticks"] = home_ticks
    calib["home_deg"] = home_deg
    calib["range_source"] = {
        "file": str(args.range),
        "recorded": sweep.get("created"),
        "samples": sweep.get("samples"),
        "duration_s": sweep.get("duration_s"),
        "observed_span_ticks": {n: sweep["range"][n]["span"] for n in JOINTS},
        "note": ("middle_ticks is the middle of the OBSERVED travel, not a placed pose. "
                 "home_ticks is a pose the operator recorded, not a computed one."),
    }
    calib["created"] = time.strftime("%Y-%m-%dT%H:%M:%S")

    if not args.write:
        print("\nDRY RUN -- nothing written. Re-run with --write to apply.")
        return

    dest = Path(args.calib)
    backup = dest.with_suffix(dest.suffix + ".bak-" + time.strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(dest, backup)
    dest.write_text(json.dumps(calib, indent=2), encoding="utf-8")
    print(f"\nbackup: {backup}")
    print(f"written: {dest}")
    print("soft calibration only -- no servo EEPROM was touched.")


if __name__ == "__main__":
    main()
