"""What do the cameras ACTUALLY deliver, one at a time and together?

The operator reported a low frame rate in the live view. Two very different causes look the
same on screen, and this script separates them:

  1. a TRANSPORT bottleneck -- USB bandwidth shared between the D435 and the wrist camera, so
     each stream starves the other;
  2. the STAGED pipeline -- a stream configured at 15 fps, plus `grab_overhead(warmup=2)`
     discarding two frames per call, plus a synchronous re-grab per /look with no caching.

Measure, do not guess. Run it with the bridge STOPPED: both cameras are exclusive devices and
the bridge owns them.

    python bridge/bench_fps.py            # both stages
    python bridge/bench_fps.py --wrist    # wrist only
    python bridge/bench_fps.py --d435     # overhead only

Notes on the numbers. The D435's colour+depth at 640x480 is requested at --fps (default 15),
so ~15 Hz is the CEILING for the overhead stage, not a fault: an aligned color+depth frame
pair is far heavier than a webcam frame. What matters for the panel is the end-to-end cost of
one /look, which is dominated by warmup discards and by the two streams being read serially.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from bridge.cameras import OverheadD435, WristCamera, dshow_device_names, index_for_device_name


def bench(label: str, grab, frames: int, warmup: int = 0) -> dict:
    for _ in range(warmup):
        grab()
    t0 = time.perf_counter()
    for _ in range(frames):
        grab()
    dt = time.perf_counter() - t0
    hz = frames / dt if dt > 0 else 0.0
    print(f"  {label:<34} {hz:6.1f} Hz   ({dt * 1000 / frames:6.1f} ms/frame)")
    return {"label": label, "hz": hz, "ms": dt * 1000 / frames}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--fps", type=int, default=15, help="D435 requested fps")
    ap.add_argument("--wrist-name", default="USB Camera")
    ap.add_argument("--wrist-only", action="store_true")
    ap.add_argument("--d435-only", action="store_true")
    args = ap.parse_args()

    results: list[dict] = []

    print("DirectShow device list (this IS the cv2 index order):")
    for i, name in enumerate(dshow_device_names()):
        print(f"  {i} -> {name}")
    print()

    d435 = wrist = None
    try:
        if not args.wrist_only:
            print(f"D435 colour+depth @ {args.fps} fps requested")
            d435 = OverheadD435(fps=args.fps).start()
            results.append(bench("D435 color+depth (aligned)", lambda: d435.get(), args.frames))

        if not args.d435_only:
            idx = index_for_device_name(args.wrist_name)
            if idx is None:
                print(f"  wrist: no device matching {args.wrist_name!r} -- skipped")
            else:
                print(f"wrist camera: {args.wrist_name!r} -> index {idx}")
                wrist = WristCamera(idx)
                # Count only frames the device actually returns, so a stalled stream reads as
                # a low rate instead of an exception.
                got = 0
                t0 = time.perf_counter()
                for _ in range(args.frames):
                    ok, frame = wrist._cap.read()
                    if ok and frame is not None:
                        got += 1
                dt = time.perf_counter() - t0
                hz = got / dt if dt > 0 else 0.0
                print(f"  {'wrist camera (raw read)':<34} {hz:6.1f} Hz   "
                      f"({dt * 1000 / max(got, 1):6.1f} ms/frame, {got}/{args.frames} frames)")
                results.append({"label": "wrist camera (raw read)", "hz": hz,
                                "ms": dt * 1000 / max(got, 1)})

        if d435 is not None and wrist is not None:
            # What /look?view=both does: read the two streams one after the other. This is the
            # number the operator actually feels.
            print("\nboth streams, read alternately (what one /look?view=both costs)")

            def both():
                d435.get()
                wrist._cap.read()

            results.append(bench("overhead + wrist, one grab each", both, max(10, args.frames // 2)))
    finally:
        if wrist is not None:
            wrist.release()
        if d435 is not None:
            d435.stop()

    print("\nsummary")
    for r in results:
        print(f"  {r['label']:<34} {r['hz']:6.1f} Hz")
    if len(results) >= 3:
        combined = results[-1]["hz"]
        print(f"\n  one /look?view=both sustains ~{combined:.1f} Hz end to end.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
