"""Write a NOMINAL overhead extrinsics from the measured mount geometry.

This is not a solved hand-eye calibration.  `calib/extrinsics.py` does that, and it
needs a ChArUco board tilted toward the camera plus measured board coordinates.  Until
somebody runs it, `/scene` has no `T_base_cam` and returns camera-frame numbers only:
no `centroid_base_mm`, no `footprint_centre_base_mm`, so no Cartesian target to grasp.

What this script does instead is reconstruct the camera pose from the numbers that were
physically measured off the bench -- where the camera sits and which way it looks -- and
write the matrix in the same file format, tagged `source` so nothing downstream can
mistake it for a solved calibration:

    eye   = [camera_forward_mm, 0, table_z_mm + camera_height_mm] / 1000
    fwd   = [-sin(pitch), 0, -cos(pitch)]      # pitched back toward the arm
    up    = [0, -1, 0]                         # image up is -y in the arm frame
    T     = look_at(eye, fwd, up)

Accuracy is whatever the tape measure was: expect tens of millimetres, and expect it to
be worst in the direction the pitch error rotates the view.  It is good enough to point
a grasp at a 45 mm block.  It is not good enough to keep.

Verify it (no intrinsics needed) by comparing an object's reported base position against
a pose the arm itself reports: with the arm at a known `tool_mm`, the arm shows up in
`arm_scene`'s object list, and that object should sit within ~20 mm of `tool_mm`.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calib.extrinsics import look_at  # noqa: E402


def build(table_z_mm: float, camera_forward_mm: float, camera_height_mm: float,
          camera_pitch_deg: float,
          up_hint: tuple[float, float, float] = (-1.0, 0.0, 0.0)) -> np.ndarray:
    eye = np.array([camera_forward_mm, 0.0, table_z_mm + camera_height_mm]) / 1000.0
    fwd = np.array([-np.sin(np.radians(camera_pitch_deg)), 0.0,
                    -np.cos(np.radians(camera_pitch_deg))])
    return look_at(eye, fwd, np.asarray(up_hint, dtype=float))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--table-z-mm", type=float, default=-115.0)
    ap.add_argument("--camera-forward-mm", type=float, default=440.0)
    ap.add_argument("--camera-height-mm", type=float, default=625.0)
    ap.add_argument("--camera-pitch-deg", type=float, default=16.0)
    ap.add_argument("--up-hint", type=float, nargs=3, default=(-1.0, 0.0, 0.0),
                    metavar=("X", "Y", "Z"),
                    help=("which way is image-up, in the arm frame.  The RealSense "
                          "is mounted on its side on this bench: image right is +y and "
                          "image down is +x, so image up is -x.  Getting this wrong "
                          "rotates the whole scene about the optical axis and still "
                          "gives a perfectly orthogonal matrix, so check it against the "
                          "table normal, which must come out near (0, 0, 1)."))
    ap.add_argument("--out", default="calib/out/overhead_extrinsics.json")
    ap.add_argument("--write", action="store_true",
                    help="actually write the file; without it this is a dry run")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing file (refuses otherwise)")
    args = ap.parse_args()

    out = ROOT / args.out
    if out.exists() and not args.force:
        print(f"{out} already exists; pass --force to replace it.  A solved "
              f"calibration is worth more than this nominal pose.", file=sys.stderr)
        return 2

    T = build(args.table_z_mm, args.camera_forward_mm, args.camera_height_mm,
              args.camera_pitch_deg, tuple(args.up_hint))

    # Report the camera's own axes in the arm frame, because a sign error here is the
    # failure this file is most likely to have, and it is invisible in the numbers.
    R = T[:3, :3]
    print(f"camera eye (arm frame, mm): {np.round(T[:3, 3] * 1000, 1).tolist()}")
    print(f"  x_cam (image right):      {np.round(R[:, 0], 4).tolist()}")
    print(f"  y_cam (image down):       {np.round(R[:, 1], 4).tolist()}")
    print(f"  z_cam (optical axis):     {np.round(R[:, 2], 4).tolist()}")
    print(f"  det(R) = {np.linalg.det(R):.6f}")

    # Where the optical axis meets the table plane: a quick sanity number, since the
    # camera was aimed by hand and should be looking at the middle of the workspace.
    t = -T[2, 3] / R[2, 2]
    look_at_table = (T[:3, 3] + R[:, 2] * t) * 1000.0
    print(f"optical axis meets table at: {np.round(look_at_table, 1).tolist()} mm")

    payload = {
        "T_base_cam": T.tolist(),
        "source": "mount_geometry_nominal",
        "note": ("Reconstructed from measured mount geometry, NOT a solved hand-eye "
                 "calibration. Expect tens of mm of error. Replace with "
                 "calib/extrinsics.py output when a board capture exists."),
        "created": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "params": {
            "table_z_mm": args.table_z_mm,
            "camera_forward_mm": args.camera_forward_mm,
            "camera_height_mm": args.camera_height_mm,
            "camera_pitch_deg": args.camera_pitch_deg,
            "up_hint": list(args.up_hint),
        },
    }
    if not args.write:
        print("\n(dry run; pass --write to create the file)")
        return 0
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
