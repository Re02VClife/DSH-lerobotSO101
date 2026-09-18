"""A synthetic overhead camera, so the whole loop can run with no hardware at all.

`bridge/sim.py` simulates the servos; this simulates the photons.  Together they let the
agent execute its full production path -- perceive, decide, plan, move, verify -- with
neither a COM port nor a RealSense attached, which is what makes it possible to validate
the stack (and the DSH tool wiring) before anything can move.

The renderer is exact rather than approximate: every pixel's ray is solved analytically
against the table plane and against each block's top face, so depth is the true
along-optical-axis value and the perception code sees the same kind of data a real camera
would give it.  Two earlier approximations here both produced errors that looked like
perception bugs: filling a block's top face with a single averaged depth (a 45 mm block
read as 117 mm, because at this mount the top face spans a large depth range) and rotating
the pixel rays by the transpose of the correct rotation (which put every object ~200 mm
from where it was).
"""
from __future__ import annotations

import numpy as np

from bridge.cameras import Intrinsics
from calib.extrinsics import look_at, invert


#: default scene: a short block, a taller one, and a flat pad, all inside the visible band
DEFAULT_BLOCKS: list[dict] = [
    {"name": "red_block", "centre_mm": (200.0, -60.0), "size_mm": (40.0, 40.0),
     "height_mm": 45.0, "bgr": (40, 40, 200)},
    {"name": "blue_block", "centre_mm": (300.0, 120.0), "size_mm": (45.0, 45.0),
     "height_mm": 60.0, "bgr": (200, 60, 40)},
    {"name": "green_pad", "centre_mm": (140.0, 60.0), "size_mm": (70.0, 70.0),
     "height_mm": 12.0, "bgr": (60, 190, 60)},
]


class SyntheticOverheadCamera:
    """Duck-typed stand-in for `bridge.cameras.OverheadD435` in simulation mode."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fx: float = 603.6943359375,
        fy: float = 602.4212646484375,
        camera_forward_mm: float = 400.0,
        camera_height_above_table_mm: float = 610.0,
        camera_pitch_deg: float = 16.0,
        table_z_mm: float = -127.4,
        blocks: list[dict] | None = None,
        noise_std_mm: float = 0.8,
        seed: int = 0,
    ) -> None:
        self.color_intr = Intrinsics(width=width, height=height, fx=fx, fy=fy,
                                     ppx=(width - 1) / 2.0, ppy=(height - 1) / 2.0,
                                     model="pinhole", coeffs=[0.0] * 5)
        self.depth_scale = 0.001
        self.depth_intr = self.color_intr
        self.table_z_mm = table_z_mm
        self.blocks = [dict(b) for b in (blocks if blocks is not None else DEFAULT_BLOCKS)]
        self.noise_std_mm = noise_std_mm
        self.rng = np.random.default_rng(seed)

        eye = np.array([camera_forward_mm, 0.0,
                        table_z_mm + camera_height_above_table_mm]) / 1000.0
        fwd = np.array([-np.sin(np.radians(camera_pitch_deg)), 0.0,
                        -np.cos(np.radians(camera_pitch_deg))])
        up_hint = np.array([0.0, -1.0, 0.0])
        self.eye_mm = eye * 1000.0
        self.T_base_cam = look_at(eye, fwd, up_hint)
        self.T_cam_base = look_at(eye, fwd, up_hint, as_cam_from_base=True)
        self.started = False

    # ---------------------------------------------------------------- lifecycle
    def start(self, *args, **kwargs) -> "SyntheticOverheadCamera":
        self.started = True
        return self

    def stop(self) -> None:
        self.started = False

    def __enter__(self) -> "SyntheticOverheadCamera":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def get(self, warmup: int = 0) -> tuple[np.ndarray, np.ndarray]:
        return self.render()

    def patch_depth(self, depth_m: np.ndarray, u: int, v: int, r: int = 3) -> float:
        h, w = depth_m.shape
        patch = depth_m[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        valid = patch[(patch > 0.05) & (patch < 3.0)]
        return float(np.median(valid)) if valid.size else 0.0

    # ------------------------------------------------------------------ render
    def render(self, blocks: list[dict] | None = None) -> tuple[np.ndarray, np.ndarray]:
        blocks = self.blocks if blocks is None else blocks
        intr = self.color_intr
        w, h = intr.width, intr.height
        R_cb = self.T_cam_base[:3, :3]
        eye = -R_cb.T @ self.T_cam_base[:3, 3]

        color = np.full((h, w, 3), (120, 120, 120), np.uint8)
        ys, xs = np.mgrid[0:h, 0:w]
        ray_cam = np.stack([(xs - intr.ppx) / intr.fx,
                            (ys - intr.ppy) / intr.fy,
                            np.ones_like(xs, dtype=float)], axis=-1)
        ray_cam /= np.linalg.norm(ray_cam, axis=-1, keepdims=True)
        ray_base = ray_cam @ R_cb                 # row-vector form of R_bc @ ray_cam

        with np.errstate(divide="ignore", invalid="ignore"):
            s_table = (self.table_z_mm / 1000.0 - eye[2]) / ray_base[..., 2]
        hit_table_base = eye + ray_base * s_table[..., None]
        hit_table_cam = (R_cb @ (hit_table_base - eye)[..., None])[..., 0]
        depth = np.asarray(hit_table_cam[..., 2], dtype=np.float32)
        depth[(s_table <= 0) | ~np.isfinite(depth)] = 0.0

        for blk in blocks:
            cx, cy = blk["centre_mm"]
            bw, bd = blk["size_mm"]
            x0, x1 = (cx - bw / 2) / 1000.0, (cx + bw / 2) / 1000.0
            y0, y1 = (cy - bd / 2) / 1000.0, (cy + bd / 2) / 1000.0
            over_block = ((hit_table_base[..., 0] >= x0) & (hit_table_base[..., 0] <= x1)
                          & (hit_table_base[..., 1] >= y0) & (hit_table_base[..., 1] <= y1))
            top_z = (self.table_z_mm + blk["height_mm"]) / 1000.0
            with np.errstate(divide="ignore", invalid="ignore"):
                s_top = (top_z - eye[2]) / ray_base[..., 2]
            hit_top_base = eye + ray_base * s_top[..., None]
            hit_top_cam = (R_cb @ (hit_top_base - eye)[..., None])[..., 0]
            on_top = over_block & (s_top > 0) & (s_top < s_table) & np.isfinite(hit_top_cam[..., 2])
            depth[on_top] = np.asarray(hit_top_cam[..., 2], dtype=np.float32)[on_top]
            color[on_top] = blk.get("bgr", (60, 60, 200))

        if self.noise_std_mm > 0:
            valid = depth > 0
            depth[valid] += self.rng.normal(0.0, self.noise_std_mm / 1000.0,
                                            int(valid.sum())).astype(np.float32)
        return color, depth
