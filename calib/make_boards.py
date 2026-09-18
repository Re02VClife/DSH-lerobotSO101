"""Generate print-ready calibration boards.

Printed size matters more than anything else in the calibration: verify the square
edge with calipers after printing, and dry-mount the sheet on a rigid plate.

BOARD 1 "table"   ChArUco, 12 x 7 squares of 30.00 mm  -> 360 x 210 mm
                  The OPERATOR'S ACTUAL BOARD, measured on the bench: 12 squares wide,
                  7 tall, 30 mm each.  450 x 300 mm is the sheet size, so the 30 mm
                  lattice occupies 360 x 210 mm of it.
                  11 x 6 = 66 inner corners.
                  ChArUco is chosen over a plain checkerboard because the arm occludes
                  part of a fixed overhead view, and a plain checkerboard returns nothing
                  at all once a corner region is covered.

BOARD 2 "wrist"   ChArUco, 8 x 11 squares of 20.00 mm   -> 160 x 220 mm
                  the 360 mm board does not fit in the wrist camera's view at grasping
                  distance, so the wrist camera needs its own target.

BOARD 3 "practice" the old 10 x 13 x 30 mm sheet, kept only so an already-printed copy
                  remains usable.

Output is written at a known DPI so that a 100 %-scale print reproduces the
physical size exactly.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2

# 10 px per mm at print time = 254 DPI
PX_PER_MM = 10.0
DPI = 25.4 * PX_PER_MM


def build(
    out: Path,
    squares_x: int,
    squares_y: int,
    square_mm: float,
    marker_ratio: float = 0.75,
    dictionary: int = cv2.aruco.DICT_5X5_100,
    margin_mm: float = 10.0,
) -> dict:
    square_m = square_mm / 1000.0
    marker_m = square_m * marker_ratio
    dic = cv2.aruco.getPredefinedDictionary(dictionary)
    board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, dic)

    board_w_mm = squares_x * square_mm
    board_h_mm = squares_y * square_mm
    margin_px = int(round(margin_mm * PX_PER_MM))
    inner_w = int(round(board_w_mm * PX_PER_MM))
    inner_h = int(round(board_h_mm * PX_PER_MM))

    img = board.generateImage((inner_w, inner_h), marginSize=0, borderBits=1)
    # white border so the printed sheet has a quiet zone for detection
    canvas = cv2.copyMakeBorder(
        img, margin_px, margin_px, margin_px, margin_px,
        cv2.BORDER_CONSTANT, value=255,
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), canvas)

    return {
        "file": str(out),
        "squares": [squares_x, squares_y],
        "inner_corners": [(squares_x - 1) * (squares_y - 1)],
        "square_mm": square_mm,
        "marker_mm": round(marker_m * 1000, 3),
        "board_mm": [round(board_w_mm, 1), round(board_h_mm, 1)],
        "sheet_mm": [round(board_w_mm + 2 * margin_mm, 1), round(board_h_mm + 2 * margin_mm, 1)],
        "image_px": [canvas.shape[1], canvas.shape[0]],
        "dpi": round(DPI, 2),
        "patternSize_for_chessboard_fallback": [squares_x - 1, squares_y - 1],
        "square_size_m_for_calibrate_camera": square_m,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="calib/boards")
    args = ap.parse_args()
    out_dir = Path(args.out_dir)

    boards = {
        "charuco_table_12x7_30mm.png": build(out_dir / "charuco_table_12x7_30mm.png", 12, 7, 30.0),
        "charuco_wrist_8x11_20mm.png": build(out_dir / "charuco_wrist_8x11_20mm.png", 8, 11, 20.0),
        "charuco_10x13_30mm.png": build(out_dir / "charuco_10x13_30mm.png", 10, 13, 30.0),
    }
    for name, info in boards.items():
        print(f"--- {name} ---")
        for k, v in info.items():
            print(f"  {k}: {v}")
    print("\nPRINT INSTRUCTIONS")
    print("  1. print at 100 % scale (do NOT use 'fit to page'); the PNGs are 254 DPI.")
    print("  2. measure the printed square edge with calipers; must be exactly 30.00 / 20.00 mm.")
    print("  3. mount on a rigid flat plate (acrylic / aluminium). Warping is a real error term.")
    print("  4. keep the sheet matte; glossy laminate creates specular glare that kills detection.")
    print("  5. for the OVERHEAD camera the board must be TILTED toward it: a board lying flat is")
    print("     viewed at ~74 deg incidence at this mount, which no corner detector recovers.")
    print("     See docs/RUNBOOK-首跑手册.md.")


if __name__ == "__main__":
    main()
