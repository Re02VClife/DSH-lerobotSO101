"""DS101 bench bridge: one local service in front of the SO-101 and both cameras.

Why a resident service instead of a script per call
---------------------------------------------------
Opening the D435 pipeline takes ~1 s and the RealSense USB 2.1 link only tolerates one
owner.  Opening COM10 while another process holds it fails outright.  So the hardware
must have exactly one owner for the whole session: this process.  Everything else --
DSH tools, a calibration script, an operator at the keyboard -- talks to it over HTTP
on loopback and gets JSON back.

Transport notes
---------------
* `http.server.ThreadingHTTPServer` from the standard library, so the bridge needs no
  third-party web dependency beyond what the robot side already needs.
* Writes are serialised by a single lock: two concurrent motion requests must never
  interleave on the bus.
* Motion is OFF unless the service was started with --allow-motion, and there is an
  explicit /estop.  A heartbeat lease releases torque if the caller disappears.

Run:
    .venv\\Scripts\\python.exe bridge\\server.py --allow-motion --port-http 8731
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.cameras import OverheadD435, WristCamera  # noqa: E402
from bridge.driver import ArmDriver, WatchdogConfig  # noqa: E402
from bridge.kinematics import LEROBOT_TO_URDF  # noqa: E402
from bridge.servo_bus import SafetyConfig, SafetyError  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


class Bench:
    """Owns the arm and the cameras.  One instance, one hardware."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        # Two locks, not one. The serial bus and the cameras are independent hardware, and
        # sharing a single lock meant a slow frame -- a D435 grab is hundreds of milliseconds,
        # and the panel polls for frames -- blocked every joint read behind it. A hand sweep
        # needs /state at 10 Hz while the panel is still fetching pictures.
        self.lock = threading.RLock()          # the serial bus
        self.cam_lock = threading.RLock()      # the RealSense and the wrist camera
        self.started_at = time.time()
        self.last_error: str | None = None
        self.capture_dir = ROOT / args.capture_dir
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        safety = SafetyConfig(
            max_step_deg=args.max_step_deg,
            max_speed_ticks_s=args.max_speed_ticks_s,
            min_voltage=args.min_voltage,
            max_temperature=args.max_temperature,
        )
        if args.simulate:
            from bridge.sim import SimServoBus, sim_calibration
            from bridge.kinematics import SO101Kinematics

            kin = SO101Kinematics()
            sim_bus = SimServoBus(allow_motion=True, safety=safety)
            self.driver = ArmDriver(
                args.port, args.calibration,
                allow_motion=True,           # the simulation is always allowed to move
                safety=safety,
                watchdog=WatchdogConfig(timeout_s=args.watchdog_s),
                calibration=sim_calibration(kin),
                bus=sim_bus,
            )
            self.sim_bus = sim_bus
            print("  MODE:        SIMULATION (no hardware is touched)")
        else:
            self.sim_bus = None
            self.driver = ArmDriver(
                args.port, args.calibration,
                allow_motion=args.allow_motion,
                safety=safety,
                watchdog=WatchdogConfig(timeout_s=args.watchdog_s),
            )
        self.driver.connect()

        self.overhead: OverheadD435 | None = None
        self.wrist: WristCamera | None = None
        self._cam_serial = args.camera_serial
        self._wrist_index = args.wrist_index
        self._wrist_name = args.wrist_name
        self._overhead_warmed = False

        # Claim the wrist camera BEFORE the RealSense, instead of lazily on the first /look.
        #
        # cv2/MSMF assigns indices from the enumerated device list at open time, and
        # pyrealsense2 claims the D435's own UVC interfaces. Opening the wrist camera lazily
        # meant it was opened AFTER the D435 was live, in a reshuffled list where the recorded
        # index pointed at a different device -- and opening that one hard-crashed the process.
        # No Python traceback, no 500, just a dead bridge: an attached camera looked unplugged.
        # Taking it first, while the list is still clean, is what makes the index mean what a
        # probe on an idle machine said it meant.
        #
        # A missing or unreadable wrist camera is not fatal -- the bench works with the
        # overhead camera alone -- so this warns and carries on rather than refusing to start.
        # Simulation is skipped entirely: the synthetic camera stands in for both.
        if not args.simulate:
            try:
                self.wrist = WristCamera(self._wrist_index, name=self._wrist_name)
                print(f"  wrist cam:   {self._wrist_name!r} -> index {self.wrist.index} "
                      f"(claimed before the D435)")
            except Exception as exc:              # noqa: BLE001 - hardware can fail any way
                print(f"  wrist cam:   unavailable ({exc})")

    # ------------------------------------------------------------- cameras
    def ensure_overhead(self) -> OverheadD435:
        if self.overhead is None:
            if self.args.simulate:
                # In simulation the camera is synthetic too, so the agent's whole loop --
                # perceive, plan, move, verify -- runs with no hardware attached.
                from bridge.fake_camera import SyntheticOverheadCamera
                cam = SyntheticOverheadCamera(table_z_mm=self.args.table_z_mm)
                cam.start()
            else:
                cam = OverheadD435(self._cam_serial, fps=self.args.fps)
                cam.start(emitter=self.args.emitter)
            self.overhead = cam
        return self.overhead

    def ensure_wrist(self) -> WristCamera:
        if self.wrist is None:
            self.wrist = WristCamera(self._wrist_index, name=self._wrist_name)
        return self.wrist

    def grab_overhead(self, warmup: int = 2) -> tuple[np.ndarray, np.ndarray]:
        """One aligned color+depth frame.

        The warmup discard happens ONCE per pipeline start, not on every grab. A RealSense
        pipeline does need a couple of frames after it starts before depth is valid, but
        re-discarding them on every single call cost ~130 ms each time -- at 15 fps one frame is
        67 ms -- which was most of the latency the live view was showing. Measured with
        bridge/bench_fps.py: the wrist camera runs at a full 29.3/30 Hz and the D435 at 12.4 of
        its requested 15. There was never a transport bottleneck to fix.
        """
        cam = self.ensure_overhead()
        if not self._overhead_warmed:
            for _ in range(warmup):
                cam.get()
            self._overhead_warmed = True
        return cam.get()

    def save_png(self, frame: np.ndarray, label: str) -> str:
        import cv2
        path = self.capture_dir / f"{label}_{int(time.time() * 1000)}.png"
        cv2.imwrite(str(path), frame)
        return str(path)

    def close(self) -> None:
        try:
            if self.overhead is not None:
                self.overhead.stop()
        except Exception:
            pass
        try:
            if self.wrist is not None:
                self.wrist.release()
        except Exception:
            pass
        try:
            self.driver.close()
        except Exception:
            pass

    # ------------------------------------------------------------ geometry
    def extrinsics(self) -> np.ndarray | None:
        if self.args.simulate and self.overhead is not None:
            # the synthetic camera knows its own pose exactly
            return getattr(self.overhead, "T_base_cam", None)
        p = ROOT / self.args.extrinsics
        if not p.exists():
            return None
        return np.array(json.loads(p.read_text(encoding="utf-8"))["T_base_cam"], dtype=float)


def jsonable(obj):
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return str(obj)


class Handler(BaseHTTPRequestHandler):
    bench: Bench = None          # set by main()
    server_version = "ds101-bridge/0.1"

    # ------------------------------------------------------------- plumbing
    def log_message(self, fmt, *args):        # keep the console readable
        return

    def _send(self, code: int, payload: dict) -> None:
        body = json.dumps(payload, default=jsonable).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                          # noqa: N802
        self._dispatch("GET")

    def do_POST(self):                         # noqa: N802
        self._dispatch("POST")

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception:
            return {}

    def _shutdown(self) -> None:
        """Stop serving first, then release the hardware best-effort.

        The order is the whole point. `close()` touches the serial port and the cameras, so when
        the reason for shutting down IS a wedged hardware call, a close-first sequence hangs
        right here and never reaches `server.shutdown()` -- leaving the process alive and still
        holding the port, which is precisely the state that forces a hard kill. Stopping the
        server first guarantees the process can exit, and process exit releases the handle
        whether or not close() got to finish.
        """
        self.server.shutdown()
        try:
            Handler.bench.close()
        except Exception:                      # noqa: BLE001 - we are already on the way out
            pass

    def _dispatch(self, method: str) -> None:
        route = urlparse(self.path).path.rstrip("/") or "/"
        args = self._read_json()
        b = Handler.bench
        try:
            # These two MUST NOT queue behind the bench lock, and that is not a style choice.
            #
            # Every other route takes that lock, because they all touch the same serial bus and
            # cameras. So when a hardware call wedges, the lock is precisely what is stuck -- and
            # a shutdown that needs the lock can never run. The only remaining way out is a hard
            # kill, and a hard kill leaves the COM port handle unreleasable until the USB adapter
            # is physically replugged. The escape hatch has to sit outside the room it escapes.
            #
            # /health reads only fields set at construction and never mutated under the lock, so
            # reading them unlocked is safe -- and it stays useful as a liveness probe exactly
            # when the bench is wedged, which is when liveness is the question.
            if route == "/health":
                return self._send(200, {
                    "ok": True,
                    "uptime_s": round(time.time() - b.started_at, 1),
                    "port": b.driver.bus.port,
                    "motion_allowed": b.driver.allow_motion,
                    "calibration": str(b.args.calibration),
                    "last_error": b.last_error,
                })

            if route == "/heartbeat":
                # Out here with /health, deliberately: this handler writes one float on the
                # driver and touches no hardware.  Served from inside the lock it queues
                # behind whatever is holding the bench -- and the case that matters most is
                # a long move, which holds the lock for its whole duration.  The lease then
                # expires while the bridge is still driving the arm, the watchdog cuts torque
                # mid-move, and the move finishes short.  A heartbeat that can be blocked by
                # the thing it is keeping alive is not a heartbeat.
                b.driver.heartbeat()
                return self._send(200, {"ok": True})

            if route == "/shutdown":
                # Answer first, then stop from a thread of its own: this request is being served
                # by a worker that shutdown() is about to wait out.
                self._send(200, {"ok": True, "stopping": True})
                threading.Thread(target=self._shutdown, daemon=True).start()
                return

            # Camera routes take the camera lock and everything else the bus lock: they do not
            # contend for the same hardware, and making them share one lock is what let a slow
            # frame stall a joint read.
            camera_routes = ("/look", "/scene", "/pixel_to_base")
            with (b.cam_lock if route in camera_routes else b.lock):
                if route == "/state":
                    return self._send(200, {"ok": True, "state": b.driver.state()})

                if route == "/scene":
                    from calib.perception import footprint_centre_base_mm, workspace_snapshot
                    color, depth = b.grab_overhead()
                    T = b.extrinsics()
                    annotate = b.capture_dir / f"scene_{int(time.time())}.png"
                    snap = workspace_snapshot(color, depth, b.overhead.color_intr,
                                              T_base_cam=T, annotate_path=annotate)
                    snap["annotated_png"] = str(annotate)
                    # the agent gets numbers; the image is for a human or a VLM later
                    snap["color_png"] = b.save_png(color, "color")

                    # geometric context the agent should not have to derive: where the camera
                    # sits in the arm frame, and the footprint centre to actually grip.
                    snap["arm_frame"] = {
                        "table_z_mm": b.args.table_z_mm,
                        "camera_position_mm": [b.args.camera_forward_mm, 0.0,
                                               b.args.table_z_mm + b.args.camera_height_mm],
                        "camera_pitch_deg": b.args.camera_pitch_deg,
                    }
                    if T is not None:
                        cam_mm = np.array([b.args.camera_forward_mm, 0.0,
                                           b.args.table_z_mm + b.args.camera_height_mm])
                        for o in snap["objects"]:
                            if o.get("centroid_base_mm") is None:
                                continue
                            # rebuild the DetectedObject views needed for the correction;
                            # the snapshot is JSON, so recompute from its fields
                            class _View:
                                centroid_base = np.array(o["centroid_base_mm"]) / 1000.0
                                height_mm = o["height_above_table_mm"]
                            g = footprint_centre_base_mm(_View(), cam_mm, b.args.table_z_mm)
                            o["footprint_centre_base_mm"] = [round(float(v), 1) for v in g]
                            o["grip_note"] = ("aim at footprint_centre_base_mm, not "
                                              "centroid_base_mm: the top face projects toward "
                                              "the camera, so the centroid is biased by height")
                    return self._send(200, {"ok": True, "scene": snap})

                if route == "/look":
                    view = args.get("view", "overhead")
                    out: dict = {}
                    if view in ("overhead", "both"):
                        color, depth = b.grab_overhead()
                        out["overhead_png"] = b.save_png(color, "overhead")
                        out["overhead_intrinsics"] = {
                            "fx": round(b.overhead.color_intr.fx, 2),
                            "fy": round(b.overhead.color_intr.fy, 2),
                            "ppx": round(b.overhead.color_intr.ppx, 2),
                            "ppy": round(b.overhead.color_intr.ppy, 2),
                        }
                    if view in ("wrist", "both"):
                        frame = b.ensure_wrist().get()
                        out["wrist_png"] = b.save_png(frame, "wrist")
                    return self._send(200, {"ok": True, "views": out})

                if route == "/pixel_to_base":
                    from calib.extrinsics import project_to_base
                    T = b.extrinsics()
                    if T is None:
                        return self._send(409, {"ok": False,
                                                "error": "no extrinsics calibration yet",
                                                "hint": f"run calib/extrinsics.py; expected {b.args.extrinsics}"})
                    color, depth = b.grab_overhead()
                    u, v = float(args["u"]), float(args["v"])
                    z = b.overhead.patch_depth(depth, int(u), int(v), r=int(args.get("radius", 4)))
                    if z <= 0:
                        return self._send(200, {"ok": False, "error": "no valid depth at that pixel"})
                    p = project_to_base(u, v, z, b.overhead.color_intr.K, T)
                    return self._send(200, {"ok": True, "depth_mm": round(z * 1000, 1),
                                            "target_mm": [round(float(x) * 1000, 1) for x in p]})

                if route == "/plan":
                    target = [float(v) / 1000.0 for v in args["target_mm"]]
                    st = b.driver.state()
                    seed = {LEROBOT_TO_URDF[k]: v for k, v in st["joint_deg"].items()
                            if k in LEROBOT_TO_URDF}
                    res = b.driver.kin.ik(np.array(target), seed)
                    return self._send(200, {"ok": res["ok"], "dry_run": True, "ik": res,
                                            "current_tool_mm": st["tool_mm"]})

                if route == "/move_to":
                    if not b.driver.allow_motion:
                        return self._send(403, {"ok": False,
                                                "error": "bridge started read-only (no --allow-motion)"})
                    b.driver.heartbeat()
                    res = b.driver.goto_tool(args["target_mm"], wait=args.get("wait", True),
                                             speed_ticks_s=args.get("speed"))
                    return self._send(200 if res.get("ok") else 409, res)

                if route == "/move_joints":
                    if not b.driver.allow_motion:
                        return self._send(403, {"ok": False,
                                                "error": "bridge started read-only (no --allow-motion)"})
                    b.driver.heartbeat()
                    res = b.driver.goto_joints(args["joints"], wait=args.get("wait", True))
                    return self._send(200 if res.get("ok") else 409, res)

                if route == "/gripper":
                    b.driver.heartbeat()
                    res = b.driver.set_gripper(args.get("openness_pct", 100))
                    return self._send(200 if res.get("ok") else 409, res)

                if route == "/home":
                    b.driver.heartbeat()
                    res = b.driver.go_home(wait=args.get("wait", True))
                    return self._send(200 if res.get("ok") else 409, res)

                if route == "/release":
                    b.driver.release()
                    return self._send(200, {"ok": True, "torque_enabled": False})

                if route == "/estop":
                    b.driver.release()
                    b.last_error = "estop"
                    return self._send(200, {"ok": True, "estopped": True,
                                            "note": "torque released; clear with /enable"})

                if route == "/enable":
                    if not b.driver.allow_motion:
                        return self._send(403, {"ok": False, "error": "read-only bridge"})
                    b.driver.enable()
                    return self._send(200, {"ok": True, "torque_enabled": True})

                if route == "/log":
                    return self._send(200, {"ok": True, "log": b.driver.log(int(args.get("limit", 50)))})

                return self._send(404, {"ok": False, "error": f"no route {route}"})

        except SafetyError as e:
            b.last_error = f"safety: {e}"
            return self._send(409, {"ok": False, "error": str(e), "kind": "safety"})
        except KeyError as e:
            return self._send(400, {"ok": False, "error": f"missing argument {e}"})
        except Exception as e:                 # noqa: BLE001 - a bridge must never die on one request
            b.last_error = f"{type(e).__name__}: {e}"
            traceback.print_exc()
            return self._send(500, {"ok": False, "error": str(e), "kind": type(e).__name__,
                                    "trace": traceback.format_exc()[-2000:]})




def preflight_port(port: str, baudrate: int = 1_000_000) -> str | None:
    """Return None when the serial port is free, else a human-readable reason.

    Added after a real incident on this bench: a force-killed bridge left the COM
    handle stuck in the kernel (the process was a zombie, so nothing user-space
    could release it) and every later script failed with a bare PermissionError
    that looked like a bug.  Failing early with the actual cause and the remedy
    saves that confusion.
    """
    import serial

    # A live bridge already owning the hardware is the common case; detect it first.
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen("http://127.0.0.1:8731/health", timeout=2) as r:
            if r.status == 200:
                return ("another bridge is already serving http://127.0.0.1:8731; use it "
                        "instead of starting a second one (the hardware allows one owner)")
    except Exception:
        pass

    try:
        s = serial.Serial(port, baudrate, timeout=0.3)
        s.close()
        return None
    except Exception as e:                       # noqa: BLE001
        return (
            f"{port} is not openable: {type(e).__name__}: {e}\n"
            "  If you see PermissionError(13) and no process is running, the handle was\n"
            "  leaked into the kernel by a killed process.  User-space cannot clear it.\n"
            "  Remedy: UNPLUG the CH343 USB serial adapter and plug it back in (or reboot).\n"
            "  Always stop a running bridge with POST /shutdown or Ctrl+C, never a hard kill."
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="DS101 bench bridge")
    ap.add_argument("--port", default="COM10", help="serial port of the SO-101 follower")
    ap.add_argument("--http-port", type=int, default=8731)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--calibration", default="calib/out/so101_calibration.json")
    ap.add_argument("--extrinsics", default="calib/out/overhead_extrinsics.json")
    ap.add_argument("--intrinsics", default="calib/out/overhead_intrinsics.json")
    ap.add_argument("--capture-dir", default="calib/captures/bridge")
    ap.add_argument("--camera-serial", default="051122071891")
    ap.add_argument("--wrist-name", default="USB Camera",
                    help="DirectShow device name of the wrist camera (substring, "
                         "case-insensitive). PREFERRED over --wrist-index: a cv2 index is a "
                         "POSITION in a list that shifts whenever a device is added, removed, or "
                         "still held by a previous process, so the same index can open the laptop "
                         "webcam on one run and the wrist camera on the next. This bench reports "
                         "'USB Camera'.")
    ap.add_argument("--wrist-index", type=int, default=None,
                    help="cv2 index of the wrist camera, used only when --wrist-name is empty. "
                         "A position is not an identity; prefer --wrist-name.")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--emitter", action="store_true",
                    help="leave the D435 IR projector on (it hurts RGB board detection, "
                         "so it is off by default)")
    ap.add_argument("--allow-motion", action="store_true",
                    help="permit any motion at all; without it the bridge is read-only")
    ap.add_argument("--simulate", action="store_true",
                    help="run against the in-memory plant in bridge/sim.py AND a synthetic "
                         "camera, instead of real hardware; motion is always permitted and "
                         "no COM port is opened")
    ap.add_argument("--table-z-mm", type=float, default=-115.0,
                    help="table surface in the arm frame; the operator measured the shoulder "
                         "axis 115 mm above the table")
    ap.add_argument("--camera-forward-mm", type=float, default=440.0,
                    help="base vertical axis to camera, measured")
    ap.add_argument("--camera-height-mm", type=float, default=625.0,
                    help="camera height above the table, measured")
    ap.add_argument("--camera-pitch-deg", type=float, default=16.0,
                    help="optical axis tilt off vertical, measured from the live depth image")
    ap.add_argument("--max-step-deg", type=float, default=15.0)
    ap.add_argument("--max-speed-ticks-s", type=int, default=600)
    ap.add_argument("--min-voltage", type=float, default=4.2)
    ap.add_argument("--max-temperature", type=int, default=65)
    ap.add_argument("--watchdog-s", type=float, default=5.0)
    ap.add_argument("--skip-port-check", action="store_true",
                    help="do not probe the serial port before starting")
    ap.add_argument("--log-file", default=None,
                    help="append stdout+stderr to this file. Needed when the bridge is started "
                         "detached (bridge/serve.ps1): a detached process has no console, so "
                         "without this its startup lines -- which wrist camera it resolved and "
                         "which index that turned out to be -- are lost exactly when they are "
                         "needed to tell a camera problem from a broken panel.")
    args = ap.parse_args()

    if args.log_file:
        log_path = Path(args.log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        # Line-buffered so a reader sees startup lines as they happen, not at process exit.
        stream = open(log_path, "a", encoding="utf-8", buffering=1)
        sys.stdout = stream
        sys.stderr = stream
        print(f"\n--- ds101 bridge starting {time.strftime('%Y-%m-%d %H:%M:%S')} ---")

    calib_path = ROOT / args.calibration
    if not args.simulate and not calib_path.exists():
        print(f"FATAL: no calibration file at {calib_path}")
        print("Run calib/arm_zero.py first. Without it, raw servo ticks are not joint")
        print("angles and every FK/IK result is silently wrong.")
        raise SystemExit(2)

    if not args.simulate and not args.skip_port_check:
        problem = preflight_port(args.port)
        if problem is not None:
            print(f"FATAL: {problem}")
            raise SystemExit(3)

    Handler.bench = Bench(args)
    srv = ThreadingHTTPServer((args.host, args.http_port), Handler)
    print(f"ds101 bridge listening on http://{args.host}:{args.http_port}")
    print(f"  serial:      {args.port}")
    print(f"  calibration: {calib_path}")
    print(f"  motion:      {'ALLOWED' if args.allow_motion else 'read-only'}")
    print("  stop with:   curl -X POST http://127.0.0.1:8731/shutdown   (or Ctrl+C)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        srv.server_close()
        Handler.bench.close()


if __name__ == "__main__":
    main()
