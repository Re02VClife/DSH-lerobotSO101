"""Camera intrinsic calibration with a ChArUco board.

OpenCV 5 note: on this machine `cv2.aruco.calibrateCameraCharuco` and
`cv2.aruco.estimatePoseCharucoBoard` do NOT exist (checked 2026-09-18), so this
module deliberately uses the portable path:

    CharucoDetector.detectBoard  ->  Board.matchImagePoints  ->  cv2.calibrateCamera

`Board.matchImagePoints` returns the object points in board coordinates together
with the matched image points, which is exactly what calibrateCamera wants, so no
CharUco-specific calibration routine is required.

The board geometry must match the printed sheet exactly. Squares are 30.00 mm on
the 300 x 400 mm board (10 x 13 squares, 108 inner corners) and 20.00 mm on the
wrist board.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np


@dataclass
class BoardSpec:
    squares_x: int
    squares_y: int
    square_m: float
    marker_ratio: float = 0.75
    dict_id: int = cv2.aruco.DICT_5X5_100

    @property
    def board(self) -> "cv2.aruco.CharucoBoard":
        dic = cv2.aruco.getPredefinedDictionary(self.dict_id)
        return cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y), self.square_m, self.square_m * self.marker_ratio, dic
        )

    @property
    def inner_corners(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)


TABLE_BOARD = BoardSpec(12, 7, 0.030)      # the operator's actual board: 360 x 210 mm
WRIST_BOARD = BoardSpec(8, 11, 0.020)
LEGACY_BOARD = BoardSpec(10, 13, 0.030)    # an older sheet, kept so it stays usable


@dataclass
class Detection:
    """One detected board in one image."""
    image_path: str
    image_size: tuple[int, int]
    n_corners: int
    n_markers: int
    coverage: float                      # fraction of image area the board spans
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None

    @property
    def range_m(self) -> float | None:
        return None if self.tvec is None else float(np.linalg.norm(self.tvec))


@dataclass
class IntrinsicResult:
    fx: float
    fy: float
    ppx: float
    ppy: float
    dist: list[float]
    rms: float
    image_size: tuple[int, int]
    n_views: int
    per_view_rms: list[float] = field(default_factory=list)

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.ppx], [0, self.fy, self.ppy], [0, 0, 1]], dtype=float)

    def as_dict(self) -> dict:
        return {
            "fx": self.fx, "fy": self.fy, "ppx": self.ppx, "ppy": self.ppy,
            "dist": list(self.dist), "rms": self.rms,
            "image_size": list(self.image_size), "n_views": self.n_views,
            "per_view_rms": self.per_view_rms,
        }


def _detector(spec: BoardSpec):
    return cv2.aruco.CharucoDetector(spec.board)


def detect_board(
    image_path: str | Path,
    spec: BoardSpec = TABLE_BOARD,
    *,
    K: np.ndarray | None = None,
    dist: np.ndarray | None = None,
) -> Detection | None:
    """Detect the board in one image; returns None when nothing useful was found."""
    img = cv2.imread(str(image_path))
    if img is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    det = _detector(spec)
    ch_corners, ch_ids, m_corners, m_ids = det.detectBoard(gray)
    if ch_corners is None or ch_ids is None or len(ch_ids) < 6:
        return None

    # board footprint in the image -> how much of the frame it covers
    pts = ch_corners.reshape(-1, 2)
    span = (pts[:, 0].max() - pts[:, 0].min()) * (pts[:, 1].max() - pts[:, 1].min())
    coverage = float(span / (w * h))

    det_out = Detection(
        image_path=str(image_path), image_size=(w, h),
        n_corners=int(len(ch_ids)), n_markers=int(len(m_ids) if m_ids is not None else 0),
        coverage=coverage,
    )
    if K is not None:
        objp, imgp = spec.board.matchImagePoints(ch_corners, ch_ids)
        if objp is not None and len(objp) >= 6:
            ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                det_out.rvec, det_out.tvec = rvec, tvec
    return det_out


def calibrate_intrinsics(
    image_paths: list[str | Path],
    spec: BoardSpec = TABLE_BOARD,
    *,
    min_corners: int = 10,
    rational_model: bool = True,
    report: bool = True,
) -> IntrinsicResult:
    """Calibrate intrinsics from a set of board images.

    Rejects views with too few corners automatically: a view with 10 of 108 corners
    constrains almost nothing and mostly adds noise.
    """
    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    used: list[str] = []
    image_size: tuple[int, int] | None = None
    det = _detector(spec)

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        ch_corners, ch_ids, _mc, _mi = det.detectBoard(gray)
        if ch_corners is None or ch_ids is None or len(ch_ids) < min_corners:
            if report:
                n = 0 if ch_ids is None else len(ch_ids)
                print(f"  skip {Path(path).name}: only {n} corners")
            continue
        objp, imgp = spec.board.matchImagePoints(ch_corners, ch_ids)
        if objp is None or len(objp) < min_corners:
            continue
        obj_points.append(objp.astype(np.float32))
        img_points.append(imgp.astype(np.float32))
        used.append(str(path))

    if len(obj_points) < 6:
        raise RuntimeError(
            f"only {len(obj_points)} usable views; need at least 6 (aim for 20-30). "
            "Capture more angles, cover all four image corners, and tilt the board +/-30 deg."
        )
    if image_size is None:
        raise RuntimeError("no readable images")

    flags = cv2.CALIB_RATIONAL_MODEL if rational_model else 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=flags
    )

    # per-view reprojection RMS, so the worst views can be dropped and re-solved
    per_view: list[float] = []
    for i, (objp, imgp) in enumerate(zip(obj_points, img_points)):
        proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, dist)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - imgp.reshape(-1, 2), axis=1).mean())
        per_view.append(round(err, 4))

    if report:
        print(f"views used: {len(obj_points)}/{len(image_paths)}")
        print(f"RMS reprojection error: {rms:.4f} px  (target < 0.3, > 1.0 = failed)")
        worst = sorted(range(len(per_view)), key=lambda i: -per_view[i])[:3]
        for i in worst:
            print(f"  worst view {Path(used[i]).name}: {per_view[i]:.3f} px")

    return IntrinsicResult(
        fx=float(K[0, 0]), fy=float(K[1, 1]), ppx=float(K[0, 2]), ppy=float(K[1, 2]),
        dist=[float(v) for v in dist.ravel()], rms=float(rms),
        image_size=image_size, n_views=len(obj_points), per_view_rms=per_view,
    )


def compare_with_factory(result: IntrinsicResult, factory: dict) -> dict:
    """Decide whether to keep the RealSense factory intrinsics or the measured ones.

    Rule of thumb: agreement within ~0.5 % of focal length means the factory values
    are good.  A larger gap (or any cropping/resizing) means use the measured set.
    """
    fx_f, fy_f = factory["fx"], factory["fy"]
    dfx = abs(result.fx - fx_f) / fx_f * 100.0
    dfy = abs(result.fy - fy_f) / fy_f * 100.0
    dppx = abs(result.ppx - factory["ppx"])
    dppy = abs(result.ppy - factory["ppy"])
    verdict = "keep factory" if (dfx < 0.5 and dfy < 0.5) else "use measured"
    return {
        "fx_diff_pct": round(dfx, 3), "fy_diff_pct": round(dfy, 3),
        "ppx_diff_px": round(dppx, 3), "ppy_diff_px": round(dppy, 3),
        "verdict": verdict,
    }


def save(result: IntrinsicResult, path: str | Path, *, camera: str, extra: dict | None = None) -> None:
    payload = {"camera": camera, "calibrated_at": None, **result.as_dict()}
    if extra:
        payload.update(extra)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(payload, indent=2), encoding="utf-8")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Calibrate camera intrinsics from board images")
    ap.add_argument("images", nargs="+", help="image paths or a directory")
    ap.add_argument("--board", choices=["table", "wrist"], default="table")
    ap.add_argument("--out", default=None)
    ap.add_argument("--factory", default=None, help="JSON with fx,fy,ppx,ppy to compare against")
    args = ap.parse_args()

    spec = TABLE_BOARD if args.board == "table" else WRIST_BOARD
    paths: list[Path] = []
    for item in args.images:
        p = Path(item)
        paths.extend(sorted(p.glob("*.png")) + sorted(p.glob("*.jpg")) if p.is_dir() else [p])

    res = calibrate_intrinsics(paths, spec)
    print(json.dumps(res.as_dict(), indent=2))
    if args.factory:
        factory = json.loads(Path(args.factory).read_text(encoding="utf-8"))
        print("vs factory:", json.dumps(compare_with_factory(res, factory), indent=2))
    if args.out:
        save(res, args.out, camera=args.board)
        print("written:", args.out)
