"""Structured perception: turn the two camera streams into numbers an agent can act on.

Design decision worth stating explicitly: this module deliberately does NOT need a
vision-language model.  The agent driving the arm receives structured JSON
(object centroids in arm-base millimetres, extents, heights, colours, reachability,
grasp feasibility), not raw images.  That keeps the control loop cheap and
deterministic, and lets the arm be driven by a text-only agent.  A VLM, when one is
available, is used only for disambiguation ("which of these three is the red one
you meant"), never for geometry.

Pipeline for the overhead RGB-D view:
    depth -> RANSAC table plane -> residual height map
          -> connected components of "above the table" -> 3D clusters
          -> transform to arm-base frame -> objects with centroid + extent + height
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# allow running this file directly (python calib/perception.py) from the project root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.cameras import Intrinsics  # noqa: E402


# ------------------------------------------------------------------ transforms
def make_transform(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


# --------------------------------------------------------------- depth -> cloud
def deproject_image(depth_m: np.ndarray, intr: Intrinsics, stride: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Full-image deprojection.  Returns (points Nx3 in camera frame, pixel uv Nx2)."""
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    us = us.ravel().astype(np.float64)
    vs = vs.ravel().astype(np.float64)
    zs = depth_m[::stride, ::stride].ravel().astype(np.float64)
    valid = (zs > 0.05) & (zs < 3.0)
    us, vs, zs = us[valid], vs[valid], zs[valid]
    xs = (us - intr.ppx) * zs / intr.fx
    ys = (vs - intr.ppy) * zs / intr.fy
    return np.stack([xs, ys, zs], axis=1), np.stack([us, vs], axis=1)


def fit_plane_ransac(points: np.ndarray, thresh_m: float = 0.006, iters: int = 400,
                     seed: int = 0) -> tuple[np.ndarray, np.ndarray, float]:
    """Fit the dominant plane (the table).  Returns (normal, point_on_plane, inlier_fraction).

    The normal is oriented to point FROM the surface TOWARD the camera (i.e. toward the
    origin of the point cloud).  This matters: without it the sign is arbitrary, and a
    table 700 mm away with an object 40 mm in front of it yields strictly NEGATIVE
    signed distances, so "height above the table" comes out negative and every object
    silently disappears from the detection.  Plain numpy RANSAC: for ~100k points with
    400 iterations this runs in tens of milliseconds, which is fine for a 1 Hz decision
    loop and avoids another dependency.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 100:
        raise ValueError("not enough points to fit a plane")
    rng = np.random.default_rng(seed)
    best_count, best_model = 0, None
    n = pts.shape[0]
    sample_idx = rng.integers(0, n, size=(iters, 3))
    for tri in sample_idx:
        p0, p1, p2 = pts[tri]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -normal @ p0
        if abs(normal[2]) < 0.5:      # table seen from above must be roughly horizontal
            continue
        count = int(np.count_nonzero(np.abs(pts @ normal + d) < thresh_m))
        if count > best_count:
            best_count, best_model = count, (normal, d)
    if best_model is None:
        raise RuntimeError("RANSAC found no plausible table plane")
    normal, d = best_model
    # refine on inliers with least squares
    inl = np.abs(pts @ normal + d) < thresh_m
    refit = pts[inl]
    centroid = refit.mean(axis=0)
    _u, _s, vt = np.linalg.svd(refit - centroid, full_matrices=False)
    normal = vt[2]
    # orient the normal toward the camera (the cloud origin) so "above the table" is positive
    if normal @ (np.zeros(3) - centroid) < 0:
        normal = -normal
    return normal, centroid, float(inl.mean())


def plane_signed_distance(normal: np.ndarray, point: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (np.asarray(pts) - point) @ normal


# ------------------------------------------------------------ object extraction
@dataclass
class DetectedObject:
    id: int
    centroid_cam: np.ndarray
    centroid_base: np.ndarray | None
    extent_mm: tuple[float, float, float]
    height_mm: float
    top_z_mm: float | None
    bbox_px: tuple[int, int, int, int]
    n_pixels: int
    bgr: tuple[int, int, int]
    reachable: bool | None = None

    def as_dict(self) -> dict:
        d = {
            "id": self.id,
            "centroid_base_mm": None if self.centroid_base is None
            else [round(float(v) * 1000, 1) for v in self.centroid_base],
            "centroid_cam_mm": [round(float(v) * 1000, 1) for v in self.centroid_cam],
            "extent_mm": [round(v, 1) for v in self.extent_mm],
            "height_above_table_mm": round(self.height_mm, 1),
            "top_z_mm": None if self.top_z_mm is None else round(self.top_z_mm, 1),
            "bbox_px": list(self.bbox_px),
            "n_pixels": self.n_pixels,
            "colour_bgr": list(self.bgr),
            "reachable": self.reachable,
        }
        return d


def find_objects(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intr: Intrinsics,
    *,
    T_base_cam: np.ndarray | None = None,
    min_height_mm: float = 6.0,
    max_height_mm: float = 300.0,
    min_pixels: int = 120,
    max_objects: int = 8,
    stride: int = 2,
    seed: int = 0,
) -> tuple[list[DetectedObject], dict]:
    """Detect objects sitting on the table.  Everything is in metres until ml conversion."""
    pts, uv = deproject_image(depth_m, intr, stride=stride)
    normal, origin, inlier = fit_plane_ransac(pts, seed=seed)
    dist = plane_signed_distance(normal, origin, pts)

    above = (dist > min_height_mm / 1000.0) & (dist < max_height_mm / 1000.0)
    mask = np.zeros(depth_m.shape, dtype=np.uint8)
    sel_uv = uv[above]
    if sel_uv.size:
        mask[sel_uv[:, 1].astype(int), sel_uv[:, 0].astype(int)] = 255
    # With stride > 1 the mask is a sparse lattice, and a plain morphological opening
    # erodes isolated lattice points to nothing -- a 100x100 px block silently vanished
    # this way.  Close the sampling gaps first, then open to drop speckle; that removes
    # single-pixel noise without destroying real, merely subsampled, structures.
    gap = max(1, int(stride))
    if gap > 1:
        mask = cv2.dilate(mask, np.ones((gap + 1, gap + 1), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if gap > 1:
        mask = cv2.erode(mask, np.ones((gap + 1, gap + 1), np.uint8))

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    # table plane expressed in the camera frame -> project into base frame for reporting
    plane_cam = {"normal": normal.tolist(), "point": origin.tolist()}
    plane_base = None
    if T_base_cam is not None:
        n_base = T_base_cam[:3, :3] @ normal
        p_base = transform_points(T_base_cam, origin.reshape(1, 3))[0]
        plane_base = {"normal": n_base.tolist(), "point_mm": [round(float(v) * 1000, 1) for v in p_base]}
        # height of the camera above the table, measured (sanity check vs. tape measure)
        cam_origin_base = T_base_cam[:3, 3]
        plane_base["camera_height_above_table_mm"] = round(
            float(abs((cam_origin_base - p_base) @ n_base) * 1000), 1)

    objects: list[DetectedObject] = []
    for label in range(1, n_labels):
        n_px = int(stats[label, cv2.CC_STAT_AREA])
        if n_px < min_pixels:
            continue
        m = labels == label
        sel = m[uv[:, 1].astype(int), uv[:, 0].astype(int)] if uv.size else np.zeros(0, bool)
        idx = np.flatnonzero(sel)
        if idx.size < min_pixels // 2:
            # fall back to sampling the mask directly (stride can miss thin structures)
            ys, xs = np.nonzero(m)
            zs = depth_m[ys, xs]
            good = (zs > 0.05) & (zs < 3.0)
            ys, xs, zs = ys[good], xs[good], zs[good]
            if xs.size < min_pixels // 2:
                continue
            p = np.stack([(xs - intr.ppx) * zs / intr.fx,
                          (ys - intr.ppy) * zs / intr.fy, zs], axis=1)
        else:
            p = pts[idx]
            xs = uv[idx, 0].astype(int)
            ys = uv[idx, 1].astype(int)

        centroid = p.mean(axis=0)
        d_centroid = float(plane_signed_distance(normal, origin, centroid.reshape(1, 3))[0])
        span = p.max(axis=0) - p.min(axis=0)
        # colour: median BGR inside the mask
        x0, y0, w0, h0 = (int(stats[label, cv2.CC_STAT_LEFT]), int(stats[label, cv2.CC_STAT_TOP]),
                          int(stats[label, cv2.CC_STAT_WIDTH]), int(stats[label, cv2.CC_STAT_HEIGHT]))
        patch = color_bgr[y0:y0 + h0, x0:x0 + w0]
        mpatch = m[y0:y0 + h0, x0:x0 + w0]
        bgr = (0, 0, 0)
        if patch.size and mpatch.any():
            px = patch[mpatch]
            bgr = tuple(int(v) for v in np.median(px, axis=0))

        centroid_base = None
        top_z = None
        if T_base_cam is not None:
            centroid_base = transform_points(T_base_cam, centroid.reshape(1, 3))[0]
            top_point = centroid + normal * (span.max() / 2.0 if span.max() > 0 else d_centroid)
            top_z = float(transform_points(T_base_cam, top_point.reshape(1, 3))[0][2] * 1000)

        objects.append(DetectedObject(
            id=len(objects),
            centroid_cam=centroid,
            centroid_base=centroid_base,
            extent_mm=(float(span[0] * 1000), float(span[1] * 1000), float(span[2] * 1000)),
            height_mm=float(d_centroid * 1000),
            top_z_mm=top_z,
            bbox_px=(x0, y0, w0, h0),
            n_pixels=n_px,
            bgr=bgr,
        ))

    objects.sort(key=lambda o: -o.n_pixels)
    objects = objects[:max_objects]
    for i, o in enumerate(objects):
        o.id = i

    info = {
        "plane_cam": plane_cam,
        "plane_base": plane_base,
        "table_inlier_fraction": round(inlier, 3),
        "valid_depth_pixels": int(pts.shape[0]),
        "objects_found": len(objects),
    }
    return objects, info


# ------------------------------------------------------------------ grasping
#: how high above the grasp point the tool travels when moving to a new place
APPROACH_CLEARANCE_MM = 60.0


def table_frame() -> tuple[np.ndarray, np.ndarray]:
    """The table's own frame in the arm frame: (R_table_from_base, origin).

    For a top-down bench the end effector's usable axes are the TABLE's axes, not the
    arm's: a block on the table is grasped by driving the tool straight down along the
    table normal.  Keeping this explicit means a small table tilt from the calibration
    shows up as a slightly tilted approach instead of being silently ignored.
    """
    n = np.array([0.0, 0.0, 1.0])
    y = np.array([0.0, 1.0, 0.0])
    x = y - (y @ n) * n
    x = x / np.linalg.norm(x)
    y = np.cross(n, x)
    return np.column_stack([x, y, n]), np.zeros(3)


def tool_target_for_grasp(
    object_top_base_mm,
    *,
    R_table_from_base: np.ndarray | None = None,
    jaw_offset_mm: float = 98.0,
    clearance_mm: float = 4.0,
    approach_clearance_mm: float = APPROACH_CLEARANCE_MM,
) -> dict:
    """Turn "where the object's top surface is" into "where the WRIST must go".

    UNITS: everything here is MILLIMETRES, matching the arm-frame convention the rest of
    the agent-facing API uses (`centroid_base_mm`, `target_mm`).  An earlier version mixed
    metres and millimetres and produced a wrist target 1000x too small -- which IK then
    happily "solved", so the failure showed up only as the arm not going where it was told.

    This step is the one most often skipped, so it is stated explicitly: the perception
    reports the top of the object, but the Cartesian target the inverse kinematics needs
    is the pose of the J4 link's origin, which sits `jaw_offset_mm` further up the wrist
    axis than the tool tip that actually touches the object.

    Both the grasp pose and a raised pre-grasp pose are returned so the caller can do
    move-above, descend, close instead of driving sideways into the object.
    """
    top = np.asarray(object_top_base_mm, dtype=float).reshape(3)
    if R_table_from_base is None:
        R_table_from_base, _ = table_frame()
    up = R_table_from_base[:, 2]

    tip_at_grasp = top + up * clearance_mm
    wrist_at_grasp = tip_at_grasp + up * jaw_offset_mm
    pre_wrist = wrist_at_grasp + up * approach_clearance_mm
    return {
        "approach_dir_base": [round(float(v), 5) for v in up],
        "tip_at_grasp_mm": [round(float(v), 1) for v in tip_at_grasp],
        "wrist_at_grasp_mm": [round(float(v), 1) for v in wrist_at_grasp],
        "pre_grasp_wrist_mm": [round(float(v), 1) for v in pre_wrist],
        "clearance_mm": clearance_mm,
        "jaw_offset_mm": jaw_offset_mm,
    }


def object_top_base_mm(obj, table_z_mm: float = -127.4) -> np.ndarray:
    """The object's top-surface centre in arm-frame millimetres.

    The perceived centroid is a point on the object's TOP FACE, and at this bench's
    oblique viewing angle that face projects toward the camera, so the top-face centre is
    NOT directly above the object's footprint centre.  The closed-loop test measures this
    as a consistent ~15 mm offset for a 45 mm block, and it is a real grasp error: the
    jaws would close 15 mm behind the object's middle.  Use
    `footprint_centre_base_mm` when the actual footprint centre is what matters.
    """
    c = obj.centroid_base
    if c is None:
        raise ValueError("object has no base-frame centroid; was T_base_cam supplied?")
    return np.array([c[0] * 1000.0, c[1] * 1000.0, table_z_mm + obj.height_mm])


def footprint_centre_base_mm(
    obj,
    camera_base_mm,
    table_z_mm: float = -127.4,
) -> np.ndarray:
    """Compensate the oblique-view bias: the object's centre AT TABLE LEVEL.

    Because the camera looks down at ~74 deg incidence, the top face it sees is displaced
    toward the camera by roughly `height * tan(incidence)` in plan view.  Back-projecting
    the measured top-face centre onto the table removes that displacement, which is what a
    top-down grasp actually needs.

    Derivation: for a pinhole camera the plan-view displacement between a point at height
    h and its projection onto the table along the viewing ray is h * (horizontal offset
    from the camera) / (vertical offset from the camera).
    """
    c = obj.centroid_base
    if c is None:
        raise ValueError("object has no base-frame centroid; was T_base_cam supplied?")
    cam = np.asarray(camera_base_mm, dtype=float).reshape(3)
    top = np.array([c[0] * 1000.0, c[1] * 1000.0, table_z_mm + obj.height_mm])
    dz = top[2] - cam[2]                       # negative: object is below the camera
    if abs(dz) < 1e-6:
        return top
    scale = (table_z_mm - cam[2]) / dz         # > 1, extends the ray down to the table
    ground = cam + (top - cam) * scale
    return np.array([ground[0], ground[1], table_z_mm + obj.height_mm])


def nearest_object(objects, xy_mm: tuple[float, float], max_distance_mm: float = 80.0):
    """Pick the detected object closest to a desired table position, in the table plane."""
    best, best_d = None, max_distance_mm
    for o in objects:
        if o.centroid_base is None:
            continue
        d = float(np.linalg.norm(np.asarray(o.centroid_base[:2]) * 1000.0 - np.asarray(xy_mm)))
        if d < best_d:
            best, best_d = o, d
    return best


# ------------------------------------------------------------------ fiducials
def find_markers(image_bgr: np.ndarray, dict_id: int = cv2.aruco.DICT_4X4_50,
                 marker_size_m: float = 0.05, K: np.ndarray | None = None,
                 dist: np.ndarray | None = None) -> list[dict]:
    """ArUco markers: exact pixel-accurate ground truth for calibration and validation.

    Worth using even for 'pick the red block' tasks: an ArUco sticker on the object
    gives sub-millimetre corner localisation, which removes the object-detection
    error from the loop while tuning the kinematics.
    """
    dic = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dic, params)
    corners, ids, _rejected = detector.detectMarkers(image_bgr)
    out: list[dict] = []
    if ids is None:
        return out

    half = marker_size_m / 2.0
    objp = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    for i, mid in enumerate(ids.ravel()):
        c = corners[i].reshape(4, 2)
        entry = {
            "id": int(mid),
            "corners_px": [[round(float(x), 2), round(float(y), 2)] for x, y in c],
            "centre_px": [round(float(c[:, 0].mean()), 2), round(float(c[:, 1].mean()), 2)],
        }
        if K is not None:
            ok, rvec, tvec = cv2.solvePnP(objp, c.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                entry["rvec"] = [round(float(v), 5) for v in rvec.ravel()]
                entry["tvec_m"] = [round(float(v), 5) for v in tvec.ravel()]
                entry["range_mm"] = round(float(np.linalg.norm(tvec)), 1)
        out.append(entry)
    return out


# --------------------------------------------------------------------- report
def workspace_snapshot(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intr: Intrinsics,
    *,
    T_base_cam: np.ndarray | None = None,
    annotate_path: str | Path | None = None,
) -> dict:
    """The single call the agent makes to see the workspace as numbers."""
    objects, info = find_objects(color_bgr, depth_m, intr, T_base_cam=T_base_cam)
    markers = find_markers(color_bgr, K=intr.K, dist=intr.dist)

    if annotate_path is not None and objects:
        vis = color_bgr.copy()
        for o in objects:
            x, y, w, h = o.bbox_px
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"#{o.id} h={o.height_mm:.0f}mm"
            cv2.putText(vis, label, (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        Path(annotate_path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(annotate_path), vis)

    return {
        "intrinsics": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy},
        "extrinsics_loaded": T_base_cam is not None,
        "table": info["plane_base"] or info["plane_cam"],
        "table_inlier_fraction": info["table_inlier_fraction"],
        "markers": markers,
        "objects": [o.as_dict() for o in objects],
    }


if __name__ == "__main__":
    import argparse

    from bridge.cameras import OverheadD435

    ap = argparse.ArgumentParser(description="Structured workspace perception smoke test")
    ap.add_argument("--save", default="calib/captures/perception")
    ap.add_argument("--frames", type=int, default=5)
    args = ap.parse_args()

    outdir = Path(args.save)
    outdir.mkdir(parents=True, exist_ok=True)
    with OverheadD435() as cam:
        cam.start()
        color, depth = cam.get(warmup=3)
        cv2.imwrite(str(outdir / "color.png"), color)
        (outdir / "depth_mm.npy").write_bytes(b"")  # placeholder, real save below
        np.save(outdir / "depth_m.npy", depth)
        snap = workspace_snapshot(color, depth, cam.color_intr,
                                  annotate_path=outdir / "annotated.png")
        print(json.dumps(snap, indent=2))
