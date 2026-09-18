"""Camera layer for the DS101 bench: overhead Intel RealSense D435 + USB wrist camera.

Verified on this machine (2026 bench):
  * D435 serial 051122071891, firmware 5.17.0.10, **negotiated on USB 2.1**.
    Consequence: colour is limited to 1920x1080@8 / 1280x720@15 / 640x480@30 and
    depth to 640x480@30.  A single 640x480@15 colour+depth pipeline plus the
    wrist camera runs at ~9 Hz end to end, which is enough for a planner loop.
    Move the D435 to a USB 3 port + USB 3 cable before doing any high-rate VLA
    policy work (~30 Hz colour + depth becomes available).
  * factory intrinsics at 640x480: colour fx=603.69 fy=602.42 ppx=316.41 ppy=234.93
    (inverse_brown_conrady), depth fx=fy=381.44 ppx=320.96 ppy=237.36
    (brown_conrady), depth scale 0.001 m.  Read them from the live profile, do
    not hardcode: they change with resolution and after firmware self-calibration.
  * overhead geometry: 55.9 deg H x 43.4 deg V colour FOV.  At 610 mm above the
    table that is a 647 x 486 mm footprint = 1.01 mm/px.  A 300 mm-wide board
    therefore fills ~46 % of the image width, and depth noise (~1-2 % of range)
    is ~6-12 mm, which dominates the calibration error budget.
  * the USB wrist camera is a plain RGB device opened with cv2 (index varies);
    resolve it by index, verify by moving the arm and checking which image moves.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    model: str
    coeffs: list[float]

    def as_dict(self) -> dict:
        return {
            "width": self.width, "height": self.height,
            "fx": self.fx, "fy": self.fy, "ppx": self.ppx, "ppy": self.ppy,
            "model": self.model, "coeffs": list(self.coeffs),
        }

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.ppx],
                         [0.0, self.fy, self.ppy],
                         [0.0, 0.0, 1.0]])

    @property
    def dist(self) -> np.ndarray:
        return np.array(self.coeffs[:5], dtype=float)


def _to_intrinsics(i) -> Intrinsics:
    return Intrinsics(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy, str(i.model), list(i.coeffs))


class OverheadD435:
    """Aligned colour+depth from the fixed overhead camera."""

    def __init__(
        self,
        serial: str | None = "051122071891",
        width: int = 640,
        height: int = 480,
        fps: int = 15,
    ) -> None:
        self.serial = serial
        self.width, self.height, self.fps = width, height, fps
        self._pipe: rs.pipeline | None = None
        self._align = rs.align(rs.stream.color)
        self.color_intr: Intrinsics | None = None
        self.depth_intr: Intrinsics | None = None
        self.depth_scale: float = 0.001

    def start(self, emitter: bool = True, auto_exposure: bool = True) -> "OverheadD435":
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self._pipe = rs.pipeline()
        profile = self._pipe.start(cfg)
        self.color_intr = _to_intrinsics(
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics())
        self.depth_intr = _to_intrinsics(
            profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics())
        stereo = profile.get_device().first_depth_sensor()
        self.depth_scale = stereo.get_depth_scale()
        stereo.set_option(rs.option.emitter_enabled, 1 if emitter else 0)
        if not auto_exposure:
            stereo.set_option(rs.option.enable_auto_exposure, 0)
        return self

    def stop(self) -> None:
        if self._pipe is not None:
            self._pipe.stop()
            self._pipe = None

    def __enter__(self) -> "OverheadD435":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def get(self, warmup: int = 0) -> tuple[np.ndarray, np.ndarray]:
        """Return (bgr, depth_m) both on the colour grid, in metres."""
        for _ in range(warmup):
            self._pipe.wait_for_frames()
        frames = self._align.process(self._pipe.wait_for_frames())
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) * self.depth_scale
        return color, depth

    def deproject(self, u: float, v: float, z_m: float) -> np.ndarray:
        """Pixel + metric Z (along the optical axis) -> point in the colour optical frame.

        Deliberately not rs.rs2_deproject_pixel_to_point so that the arithmetic is
        explicit: depth from an aligned map is Z along the optical axis, NOT range.
        """
        if self.color_intr is None:
            raise RuntimeError("call start() first")
        K = self.color_intr
        return np.array([(u - K.ppx) * z_m / K.fx,
                         (v - K.ppy) * z_m / K.fy,
                         z_m])

    def patch_depth(self, depth: np.ndarray, u: int, v: int, r: int = 3) -> float:
        """Robust depth for a pixel: median of the valid neighbourhood, in metres."""
        h, w = depth.shape
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1]
        valid = patch[(patch > 0.05) & (patch < 3.0)]
        return float(np.median(valid)) if valid.size else 0.0


def dshow_device_names() -> list[str]:
    """DirectShow input devices, in the order cv2's CAP_DSHOW enumerates them.

    This exists because a cv2 index is a POSITION, not an identity. The DSHOW list is not
    stable: a device still held by a previous process, or briefly absent, drops out and every
    later entry shifts down one. This bench has five entries (wrist camera, laptop webcam, a
    phone, and the D435's depth + RGB UVC interfaces), so one missing device is enough to turn
    `index 0` from the wrist camera into the laptop webcam -- which is exactly what happened:
    the panel displayed a person's shirt while the arm had not moved at all.

    Names come from the same enumerator cv2 uses, so a position in this list IS the cv2 index.
    An empty list means pygrabber is unavailable and the caller should fall back to an index.
    """
    try:
        from pygrabber.dshow_graph import FilterGraph
    except ImportError:
        return []
    try:
        return [str(name) for name in FilterGraph().get_input_devices()]
    except Exception:                          # noqa: BLE001 - COM can fail in many ways
        return []


def index_for_device_name(name: str, names: list[str] | None = None) -> int | None:
    """cv2 index of the first DirectShow device whose name contains `name`, case-insensitively.

    Returns None when nothing matches, which the caller must treat as an error: silently
    falling back to a position is the bug this function exists to remove.
    """
    pool = dshow_device_names() if names is None else names
    needle = name.strip().lower()
    if not needle:
        return None
    for i, device in enumerate(pool):
        if needle in device.lower():
            return i
    return None


class WristCamera:
    """Plain USB RGB camera on the wrist.

    The cv2 index is machine-dependent AND order-dependent. Devices are numbered from the list
    a capture backend enumerates when it opens, and pyrealsense2's claim on the D435's UVC
    interfaces changes that list -- so an index that worked on an idle machine can point at a
    different device once the RealSense is live. The default below is what an idle probe found
    on the bench this was written for; `probe()` reports the current mapping, and the bridge
    opens this camera BEFORE the RealSense precisely so the number keeps meaning that.

    CAP_DSHOW, not CAP_MSMF -- a 56x difference, not a preference. On this bench MSMF takes
    ~31 s to open this camera and then spends ~33 s on EACH property set, because every one
    triggers a full format renegotiation: the four sets below cost ~130 s before a single frame
    arrives. The bridge therefore looked hung -- burning CPU, no output, no listener -- and the
    first instinct was a dead camera. DirectShow opens the same device in 0.56 s and reads a
    frame in 0.07 s.
    """

    def __init__(self, index: int | None = 0, width: int = 640, height: int = 480, fps: int = 30,
                 backend: int = cv2.CAP_DSHOW, name: str | None = None) -> None:
        # Resolve by NAME when one is given; see dshow_device_names() for why a position is not
        # an identity on this bench. An unmatched name is a hard error rather than a fallback:
        # quietly opening some other camera is what produced a panel showing a person's shirt.
        self.requested_name = name
        if name is not None:
            found = index_for_device_name(name)
            if found is None:
                raise RuntimeError(
                    f"no DirectShow device matching {name!r}; present: {dshow_device_names()}. "
                    f"Pass --wrist-index to select by position instead."
                )
            index = found
        if index is None:
            raise RuntimeError("wrist camera needs either a device name or an index")
        self.index, self.width, self.height, self.fps = index, width, height, fps
        self._cap = cv2.VideoCapture(index, backend)
        if not self._cap.isOpened():
            raise RuntimeError(f"wrist camera index {index} did not open")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def get(self, attempts: int = 6, settle_s: float = 0.05) -> np.ndarray:
        """One frame, retried.

        A UVC sensor needs a few frames after its stream opens before it produces a usable one,
        and a single read() on a cold device is the classic way to get a spurious "no frame".
        That is exactly what happened here: the first /look of a session failed while every
        later one succeeded, which reads as an intermittent hardware fault and is not one.
        """
        for _ in range(max(1, attempts)):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                return frame
            time.sleep(settle_s)
        raise RuntimeError(
            f"wrist camera at index {self.index} opened but produced no frame in {attempts} "
            f"attempts. The cv2 index is machine- and ORDER-dependent -- a capture backend "
            f"numbers devices from the list it enumerates at open time, and the RealSense "
            f"reshuffles that list -- so an index verified on an idle machine can point "
            f"elsewhere once the D435 is live. Run calib/out/probe_wrist.py with the bridge "
            f"stopped to see the current mapping."
        )
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        return frame

    def release(self) -> None:
        self._cap.release()

    def __enter__(self) -> "WristCamera":
        return self

    def __exit__(self, *exc) -> None:
        self.release()


def probe(indices: tuple[int, ...] = (0, 1, 2, 3, 4)) -> dict:
    """Report which cv2 index is which camera. Run before trusting any index."""
    out: dict[str, object] = {"d435": None, "usb": []}
    ctx = rs.context()
    out["d435"] = [{"name": d.get_info(rs.camera_info.name),
                    "serial": d.get_info(rs.camera_info.serial_number),
                    "firmware": d.get_info(rs.camera_info.firmware_version),
                    "usb": d.get_info(rs.camera_info.usb_type_descriptor)} for d in ctx.query_devices()]
    for idx in indices:
        # DSHOW for the latency reason in WristCamera's docstring: an MSMF sweep of six indices
        # takes minutes and reads exactly like a hang.
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ok, frame = cap.read()
            if ok and frame is not None:
                out["usb"].append({"index": idx, "shape": list(frame.shape),
                                   "mean_brightness": round(float(frame.mean()), 1)})
        cap.release()
    return out


if __name__ == "__main__":
    print(json.dumps(probe(), indent=2))
    with OverheadD435() as cam:
        t0 = time.time()
        n = 0
        while time.time() - t0 < 3.0:
            color, depth = cam.get()
            n += 1
        print(f"D435 {cam.width}x{cam.height}@{cam.fps}: {n/3.0:.1f} Hz, depth_scale={cam.depth_scale}")
        print("colour intrinsics:", json.dumps(cam.color_intr.as_dict(), indent=2))
        print("depth intrinsics:", json.dumps(cam.depth_intr.as_dict(), indent=2))
        z = cam.patch_depth(depth, cam.width // 2, cam.height // 2, r=8)
        print("centre patch depth (m):", round(z, 4))
        if z > 0:
            print("centre deprojected (camera optical frame, m):", np.round(cam.deproject(cam.width/2, cam.height/2, z), 4))
