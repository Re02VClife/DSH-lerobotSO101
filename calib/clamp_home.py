"""Move HOME inside the motion envelope, so `arm_home` can actually reach it.

Why this is needed. Home is a pose the OPERATOR recorded by hand, and a hand-recorded pose can
sit right on a hard limit. On this bench three joints did:

    shoulder_lift  75.54 deg, 0.97 deg off its measured lower limit (74.57)
    elbow_flex    271.77 deg, 0.70 deg off its measured upper limit (272.47)
    gripper       164.48 deg, 2.73 deg off its measured lower limit (161.75)

The motion envelope keeps `safety.limit_margin_deg` (6 deg) clear of the measured limits, so
every one of those joints had its home target clamped and `arm_home` could only answer:

    "target outside the joint envelope; the arm moved as far as allowed"

-- the firmware refusing to reach the pose its own operator had recorded.

What this does: moves each joint by the SMALLEST amount that puts it inside the envelope, so the
pose barely changes and the operator keeps the reference they chose. It rewrites `home_deg` AND
`home_ticks`, because `go_home()` drives from `home_ticks`; updating only the degrees would
leave the actual motion target unchanged and the fix invisible.

Dry run by default. Writes a timestamped backup before touching anything.

    python calib/clamp_home.py                       # show what would change
    python calib/clamp_home.py --write               # apply
    python calib/clamp_home.py --margin-deg 6 --pad-deg 0.5
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIB = ROOT / "calib" / "out" / "so101_calibration.json"

TICKS_PER_TURN = 4096.0
DEG_PER_TICK = 360.0 / TICKS_PER_TURN


def deg_to_ticks(deg: dict[str, float], calib: dict) -> dict[str, int]:
    """Invert the calibration's own conversion, using its own middle pose and sign.

    deg = middle_pose_deg + (tick - middle_ticks) * DEG_PER_TICK * sign
    """
    mid = calib["middle_ticks"]
    mid_deg = calib["middle_pose_deg"]
    sign = calib.get("direction_sign", {})
    out: dict[str, int] = {}
    for name, value in deg.items():
        s = sign.get(name, 1) or 1
        out[name] = int(round(mid[name] + (value - mid_deg[name]) / (DEG_PER_TICK * s)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--margin-deg", type=float, default=6.0,
                    help="must match bridge/servo_bus.py SafetyConfig.limit_margin_deg, or the "
                         "result will be clamped again for a different reason")
    ap.add_argument("--pad-deg", type=float, default=0.5,
                    help="extra slack so the result survives tick quantisation and float "
                         "rounding; a value that lands exactly on the boundary can round back "
                         "outside it")
    ap.add_argument("--write", action="store_true", help="apply (default is a dry run)")
    args = ap.parse_args()

    path = Path(args.calib)
    calib = json.loads(path.read_text(encoding="utf-8"))

    limits = calib.get("joint_limits_deg")
    home = calib.get("home_deg")
    if not limits or not home:
        print("FATAL: calibration has no joint_limits_deg / home_deg; run apply_range.py first.")
        return 2

    budget = args.margin_deg + args.pad_deg
    new_home: dict[str, float] = {}
    moves: list[str] = []
    for name, value in home.items():
        span = limits.get(name)
        if not span:
            new_home[name] = value
            continue
        lo, hi = float(span[0]) + budget, float(span[1]) - budget
        if lo > hi:
            print(f"FATAL: {name} has no room left: measured span {span} is narrower than "
                  f"2 x {budget} deg. Lower the margin or re-measure the joint.")
            return 3
        target = max(lo, min(hi, float(value)))
        if abs(target - float(value)) > 1e-9:
            moves.append(f"  {name:<14} {value:7.2f} -> {target:7.2f}  "
                         f"({target - value:+.2f} deg)")
        new_home[name] = round(target, 3)

    print(f"calibration : {path}")
    print(f"envelope    : measured limits +/- {args.margin_deg} deg "
          f"(+{args.pad_deg} deg pad = {budget} deg)")
    if not moves:
        print("\nHOME is already inside the envelope; nothing to do.")
        return 0

    print(f"\n{len(moves)} joint(s) would move the minimum distance to become reachable:")
    print("\n".join(moves))

    old_ticks = calib.get("home_ticks", {})
    new_ticks = deg_to_ticks(new_home, calib)
    print("\nresulting home_ticks:")
    for name in new_home:
        before = old_ticks.get(name)
        after = new_ticks[name]
        delta = f"{after - before:+d}" if isinstance(before, int) else "n/a"
        print(f"  {name:<14} {before!s:>6} -> {after:>6}  ({delta} ticks)")

    if not args.write:
        print("\ndry run: nothing written. Re-run with --write to apply.")
        return 0

    backup = path.with_suffix(path.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(path, backup)
    calib["home_deg"] = new_home
    calib["home_ticks"] = new_ticks
    calib["home_source"] = {
        "note": ("home_deg/home_ticks were moved the minimum distance inside the motion "
                 "envelope by calib/clamp_home.py; the operator's original pose sat closer to a "
                 "hard limit than safety.limit_margin_deg allows, so arm_home could never reach "
                 "it."),
        "margin_deg": args.margin_deg,
        "pad_deg": args.pad_deg,
        "moved": moves,
        "recorded": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    path.write_text(json.dumps(calib, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {path}\nbackup {backup.name}")
    print("Restart the bridge for the new home_ticks to take effect.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
