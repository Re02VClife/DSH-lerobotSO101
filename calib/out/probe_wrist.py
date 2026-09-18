"""One-off probe: which cv2/MSMF index is the wrist camera?

`bridge/cameras.py` ships `probe()`, but it only reports frame shape and mean brightness,
which cannot tell the wrist camera apart from the laptop's built-in webcam -- both are
640x480 UVC devices. So this saves one frame per index instead, and whoever is looking can
just look.

Run it with the bridge STOPPED. The bridge now claims the wrist camera at start-up, so it is
already holding this device while it runs -- and an index pointing at a device someone else
owns fails with E_INVALIDARG (-2147024809), which makes an attached camera look unplugged.

DSHOW, not MSMF: on this bench MSMF spends ~31 s opening this camera and ~33 s on each
property set, so an MSMF sweep of six indices takes minutes and reads as a hang.
"""
from __future__ import annotations

from pathlib import Path

import cv2

OUT = Path(__file__).resolve().parent / "wrist_probe"
OUT.mkdir(parents=True, exist_ok=True)

print("opencv", cv2.__version__)
for index in range(6):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"index {index}: did not open")
        cap.release()
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # A UVC sensor hands back its first usable frame only after it has warmed up, so drain a
    # few rather than trusting the very first read.
    frame = None
    for _ in range(6):
        ok, candidate = cap.read()
        if ok and candidate is not None:
            frame = candidate
    cap.release()
    if frame is None:
        print(f"index {index}: opened but produced no frame")
        continue
    path = OUT / f"idx_{index}.png"
    cv2.imwrite(str(path), frame)
    print(f"index {index}: shape={frame.shape} mean={float(frame.mean()):.1f} -> {path}")
