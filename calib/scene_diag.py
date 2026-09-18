"""Diagnose the overhead camera's view of the table: height, tilt, usable grasp area.

Run this with the table CLEAR (nothing on it) to get the geometric ground truth the
calibration depends on.  It reports, in the arm's own frame:

  * the table plane (height and tilt) as seen by the camera, using factory intrinsics
    so it works before any calibration exists;
  * where the arm's reachable annulus (from FK, no calibration needed) lands in the
    camera image, so you can see how much of the frame is actually usable.

Usage:
    .venv\\Scripts\\python.exe calib\\scene_diag.py --table-z-mm -127
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.cameras import OverheadD435  # noqa: E402
from bridge.kinematics import ALL_JOINTS, SO101Kinematics  # noqa: E402
from calib.perception import deproject_image, fit_plane_ransac, plane_signed_distance  # noqa: E402


def table_report(depth_m: np.ndarray, intr, stride: int = 2) -> dict:
    pts, uv = deproject_image(depth_m, intr, stride=stride)
    normal, origin, inlier = fit_plane_ransac(pts)
    dist = plane_signed_distance(normal, origin, pts)
    # residual spread of the table points = how flat the surface really looks
    inl = np.abs(dist) < 0.008
    tilt_deg = float(np.degrees(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0))))
    return {
        "plane_normal_cam": [round(float(v), 4) for v in normal],
        "plane_point_cam_mm": [round(float(v) * 1000, 1) for v in origin],
        "inlier_fraction": round(float(inlier), 3),
        "tilt_of_table_rel_camera_deg": round(tilt_deg, 2),
        "points_used": int(pts.shape[0]),
        "residual_p95_mm": round(float(np.percentile(np.abs(dist[inl]), 95) * 1000), 2),
        "_internal": {"normal": normal, "origin": origin, "pts": pts, "uv": uv, "dist": dist},
    }


def reach_projection(kin: SO101Kinematics, intr, table_z_mm: float, camera_xy_mm: tuple[float, float]) -> dict:
    """Approximate where the reachable annulus sits in the image.

    This uses the *pin-hole geometry only*: it projects candidate tool positions into
    the image assuming the camera sits at `camera_xy_mm` looking straight down from the
    far corner at height `camera_h_mm + table_z_mm`.  It is a planning aid to decide
    camera placement, not a calibration.
    """
    mirror = kin.base_offset_m * 0.0 + np.array([0.0, 0.0, 0.0])
    mirror[2] = table_z_mm / 1000.0
    q_mid = {n: (kin.limits_deg[n][0] + kin.limits_deg[n][1]) / 2 for n in ALL_JOINTS}
    hits = []
    for r in np.linspace(0.05, 0.45, 25):
        for a in np.linspace(-np.pi, np.pi, 72):
            target = np.array([r * np.cos(a), r * np.sin(a), table_z_mm / 1000.0])
            res = kin.ik(target, q_mid, max_iters=60)
            if res["ok"]:
                hits.append(target)
    if not hits:
        return {"reachable_table_points": 0}
    hits = np.array(hits)
    return {
        "reachable_table_points": len(hits),
        "reach_radius_mm": [round(float(np.linalg.norm(hits[:, :2], axis=1).min()) * 1000, 1),
                            round(float(np.linalg.norm(hits[:, :2], axis=1).max()) * 1000, 1)],
        "reach_x_range_mm": [round(float(hits[:, 0].min()) * 1000, 1), round(float(hits[:, 0].max()) * 1000, 1)],
        "reach_y_range_mm": [round(float(hits[:, 1].min()) * 1000, 1), round(float(hits[:, 1].max()) * 1000, 1)],
        "camera_xy_assumed_mm": list(camera_xy_mm),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--table-z-mm", type=float, default=-127.4,
                    help="height of the table in the arm frame; measure it")
    ap.add_argument("--camera-x-mm", type=float, default=400.0)
    ap.add_argument("--camera-y-mm", type=float, default=0.0)
    ap.add_argument("--save", default="calib/captures/scene_diag")
    args = ap.parse_args()

    outdir = Path(args.save)
    outdir.mkdir(parents=True, exist_ok=True)

    with OverheadD435() as cam:
        cam.start()
        color, depth = cam.get(warmup=5)
        cv2.imwrite(str(outdir / "color.png"), color)
        np.save(outdir / "depth_m.npy", depth)
        rep = table_report(depth, cam.color_intr)

    intr = cam.color_intr
    internal = rep.pop("_internal")
    normal, origin = internal["normal"], internal["origin"]

    # pixel footprint of the table region and the range of depths across the frame
    dist = internal["dist"]
    table_mask = np.abs(dist) < 0.008
    uv = internal["uv"][table_mask]
    if uv.size:
        rep["table_pixel_bbox"] = [int(uv[:, 0].min()), int(uv[:, 1].min()),
                                   int(uv[:, 0].max()), int(uv[:, 1].max())]
    valid = depth[(depth > 0.05) & (depth < 3.0)]
    rep["depth_range_mm"] = [round(float(valid.min()) * 1000, 1), round(float(valid.max()) * 1000, 1)]
    rep["depth_median_mm"] = round(float(np.median(valid)) * 1000, 1)

    kin = SO101Kinematics()
    rep["arm_reach"] = reach_projection(kin, intr, args.table_z_mm, (args.camera_x_mm, args.camera_y_mm))
    rep["colour_intrinsics"] = {"fx": round(intr.fx, 2), "fy": round(intr.fy, 2),
                               "ppx": round(intr.ppx, 2), "ppy": round(intr.ppy, 2)}
    rep["scale_at_table_mm_per_px"] = round(
        abs(args.table_z_mm - 610.0) / intr.fx if args.table_z_mm else 610.0 / intr.fx, 3)

    print(json.dumps(rep, indent=2))
    (outdir / "scene_diag.json").write_text(json.dumps(rep, indent=2), encoding="utf-8")
    print("\nsaved:", outdir)


if __name__ == "__main__":
    main()
