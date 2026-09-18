"""Sweep-check a Cartesian move before issuing it.

`arm_plan` answers "can IK reach this point?" and nothing else.  What it does not answer
is where the tool goes on the way there.  The driver's trajectory generator walks the
joints toward the target at up to 15 deg per command, which is essentially a straight
line in *joint* space -- and a straight line in joint space is not a straight line in
Cartesian space.  A move whose start and end are both 65 mm clear of the table can still
dip the gripper through it when a wrist reconfigures by 119 degrees on the way.

This takes the present pose from the running bridge, solves IK for the target, walks the
joint-space line, and reports the tool trace: minimum height above the table, the radius
range, and the largest single-joint swing.  It touches no hardware and moves nothing.

    python bridge/path_check.py --target 242.9 27.4 -50

Exit status is 1 if the swept path drops below `--clearance-mm` above the table, so it
can be used as a gate in a script.
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.kinematics import (  # noqa: E402
    ARM_JOINTS,
    LEROBOT_TO_URDF,
    SO101Kinematics,
    URDF_TO_LEROBOT,
)

#: ARM_JOINTS is keyed by URDF name; /state and IK both speak lerobot names.
LEROBOT_ARM = [URDF_TO_LEROBOT[n] for n in ARM_JOINTS]


def fetch_state(http: str) -> dict:
    req = urllib.request.Request(f"{http}/state", data=b"{}",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as fh:
        return json.loads(fh.read().decode("utf-8"))["state"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--target", type=float, nargs=3, required=True,
                    metavar=("X", "Y", "Z"), help="target tool point, arm-frame mm")
    ap.add_argument("--http", default="http://127.0.0.1:8731")
    ap.add_argument("--table-z-mm", type=float, default=-115.0)
    ap.add_argument("--clearance-mm", type=float, default=5.0,
                    help="fail if the swept tool point comes closer than this to the table")
    ap.add_argument("--steps", type=int, default=160)
    args = ap.parse_args()

    st = fetch_state(args.http)
    start = st["joint_deg"]
    kin = SO101Kinematics()

    seed = {LEROBOT_TO_URDF[k]: v for k, v in start.items() if k in LEROBOT_TO_URDF}
    res = kin.ik(np.array(args.target, dtype=float) / 1000.0, seed)
    if not res.get("ok"):
        print(f"IK failed: {res.get('reason')}", file=sys.stderr)
        return 2
    goal = res["q_deg"]

    swing = {k: goal[k] - start[k] for k in LEROBOT_ARM}
    worst = max(swing, key=lambda k: abs(swing[k]))
    print(f"IK ok, residual {res['residual_mm']:.2f} mm, {res['iters']} iters")
    print(f"largest joint swing: {worst} {swing[worst]:+.1f} deg")
    print(f"present tool: {[st['tool_mm'][0], st['tool_mm'][1], st['tool_mm'][2]]}")
    print(f"target  tool: {list(args.target)}")

    pts = []
    for i in range(args.steps + 1):
        t = i / args.steps
        q = {k: start[k] + t * swing[k] for k in LEROBOT_ARM}
        T = kin.fk({LEROBOT_TO_URDF[k]: v for k, v in q.items()})
        pts.append(np.asarray(T[:3, 3], dtype=float) * 1000.0)
    pts = np.asarray(pts)

    z = pts[:, 2]
    radius = np.hypot(pts[:, 0], pts[:, 1])
    floor = args.table_z_mm + args.clearance_mm
    i_min = int(np.argmin(z))
    print(f"\nminimum tool z along the path: {z.min():.1f} mm at step {i_min} "
          f"(x={pts[i_min, 0]:.1f}, y={pts[i_min, 1]:.1f})")
    print(f"table is at {args.table_z_mm:.1f} mm, required clearance "
          f"{(floor - args.table_z_mm):.1f} mm -> floor {floor:.1f} mm")
    print(f"radius range along the path: {radius.min():.1f} .. {radius.max():.1f} mm")
    print("\nsampled trace (tool x, y, z):")
    for i in range(0, args.steps + 1, max(1, args.steps // 8)):
        print(f"  t={i / args.steps:4.2f}  "
              f"x={pts[i, 0]:7.1f} y={pts[i, 1]:7.1f} z={pts[i, 2]:7.1f}")

    if z.min() < floor:
        print(f"\nREFUSED: the swept path goes {floor - z.min():.1f} mm below the "
              f"required clearance.  Add a waypoint -- lift straight up first, traverse "
              f"high, descend last -- rather than accepting the dip.", file=sys.stderr)
        return 1
    print("\npath clears the table.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
