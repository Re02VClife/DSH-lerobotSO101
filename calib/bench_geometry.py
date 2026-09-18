"""Check the calibration framing against the operator's REAL bench measurements.

Run this before a capture session: it answers "will the board fit in the frame" from
mount numbers alone, so a session is never wasted.

Measured on the bench (operator, 2026-09-18, tape measure, +/- a few mm):
    camera height above the table      625 mm
    base vertical axis -> camera       440 mm
    shoulder joint axis above table    115 mm
    board                              12 x 7 squares of 30 mm (360 x 210 mm)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from calib.extrinsics import framing_check, look_at, visible_table_band  # noqa: E402
from calib.intrinsics import TABLE_BOARD, WRIST_BOARD  # noqa: E402

# ---- the operator's measurements -------------------------------------------
CAM_HEIGHT_MM = 625.0
CAM_FORWARD_MM = 440.0
SHOULDER_ABOVE_TABLE_MM = 115.0
TABLE_Z_MM = -SHOULDER_ABOVE_TABLE_MM     # the table in the arm frame
CAM_PITCH_DEG = 16.0                      # measured from the live depth image


def main() -> None:
    print(f"attach: table at z={TABLE_Z_MM:.1f} mm (shoulder axis is {SHOULDER_ABOVE_TABLE_MM:.0f} mm above it)")
    print(f"camera: {CAM_FORWARD_MM:.0f} mm forward, {CAM_HEIGHT_MM:.0f} mm above the table, "
          f"{CAM_PITCH_DEG:.0f} deg off vertical\n")

    # where the camera sits in the arm frame, and how far it is from a chip on the table
    eye = np.array([CAM_FORWARD_MM, 0.0, TABLE_Z_MM + CAM_HEIGHT_MM]) / 1000.0
    T_cam_base = look_at(eye, np.array([-np.sin(np.radians(CAM_PITCH_DEG)), 0.0,
                                        -np.cos(np.radians(CAM_PITCH_DEG))]),
                         np.array([0.0, -1.0, 0.0]), as_cam_from_base=True)
    band = visible_table_band(T_cam_base, TABLE_Z_MM)
    print("visible table band (arm frame):")
    print(f"  x: {band['x_range_mm']} mm")
    print(f"  y: {band['y_range_mm']} mm")
    print(f"  depth of the visible strip: {band['depth_of_visible_band_mm']} mm")
    print(f"  working radius the arm can reach: about 350-430 mm\n")

    print(f"board: {TABLE_BOARD.squares_x} x {TABLE_BOARD.squares_y} squares of "
          f"{TABLE_BOARD.square_m * 1000:.0f} mm = "
          f"{TABLE_BOARD.squares_x * TABLE_BOARD.square_m * 1000:.0f} x "
          f"{TABLE_BOARD.squares_y * TABLE_BOARD.square_m * 1000:.0f} mm, "
          f"{TABLE_BOARD.inner_corners} inner corners\n")

    cases = [
        ("flat on the table, x=200", 200.0, -100.0, 0.0),
        ("flat on the table, x=300", 300.0, -100.0, 0.0),
        ("tilted 45 deg,        x=150", 150.0, -100.0, 45.0),
        ("tilted 50 deg,        x=120", 120.0, -100.0, 50.0),
        ("tilted 55 deg,        x=100", 100.0, -120.0, 55.0),
        ("tilted 60 deg,        x=80 ", 80.0, -120.0, 60.0),
    ]
    print(f"{'placement':28s} {'fits':>5s}  {'pixel bbox':30s} verdict")
    for label, bx, by, pitch in cases:
        r = framing_check(TABLE_BOARD, (bx, by, TABLE_Z_MM), 0.0, pitch, TABLE_Z_MM,
                          CAM_FORWARD_MM, CAM_HEIGHT_MM, CAM_PITCH_DEG)
        bbox = r["board_pixel_bbox"]
        bbox_s = "n/a" if bbox is None else f"[{bbox[0]:.0f},{bbox[1]:.0f}]..[{bbox[2]:.0f},{bbox[3]:.0f}]"
        print(f"{label:28s} {str(r['board_fully_inside_frame']):>5s}  {bbox_s:30s} "
              f"{'' if r['board_fully_inside_frame'] else r['verdict'][:44]}")

    print("\nwrist board (160 x 220 mm) is for the wrist camera, not this one.\n")
    print("RECOMMENDATION")
    print("  A board lying flat is viewed at ~74 deg incidence at this mount, which no corner")
    print("  detector recovers. Tilt it 45-60 deg toward the camera and place it where the")
    print("  table above is printed 'fits=True'. The arm will be in the frame too, which is")
    print("  why the board is ChArUco: a plain checkerboard returns nothing once occluded.")


if __name__ == "__main__":
    main()
