"""Tile geometry of the scan: must match firmware main/scan.c and config.h.
test_data_tools.py compiles scan.c and compares both.

The camera frame is CAM_W x CAM_H; the model sees SCAN_COLS x SCAN_ROWS overlapping
TILE x TILE windows. Positions are in [-1, 1]: tile-local for the model, frame-wide for labels.
"""
import numpy as np

CAM_W, CAM_H, TILE, SCAN_COLS, SCAN_ROWS = 320, 240, 96, 4, 3
SCAN_TILES = SCAN_COLS * SCAN_ROWS
THERMAL_DIV = 2           # the thermal frame given to the model is CAM_W/2 x CAM_H/2 (config.h THERMAL_DIV)
GROUND_PER_ALT = 0.86     # ground seen along the frame height per metre of altitude (config.h CAM_GROUND_PER_ALT)


def origin(t):
    col, row = t % SCAN_COLS, t // SCAN_COLS
    x0 = (col * (CAM_W - TILE) + (SCAN_COLS - 1) // 2) // (SCAN_COLS - 1) if SCAN_COLS > 1 else 0
    y0 = (row * (CAM_H - TILE) + (SCAN_ROWS - 1) // 2) // (SCAN_ROWS - 1) if SCAN_ROWS > 1 else 0
    return x0, y0


ORIGINS = [origin(t) for t in range(SCAN_TILES)]


def tile_to_frame(t, tx, ty):
    x0, y0 = ORIGINS[t]
    return (x0 + (tx + 1) * 0.5 * TILE) / (CAM_W * 0.5) - 1, (y0 + (ty + 1) * 0.5 * TILE) / (CAM_H * 0.5) - 1


def frame_to_tile(t, fx, fy):
    """Frame position -> position inside tile t (outside the tile gives |value| > 1)."""
    x0, y0 = ORIGINS[t]
    px, py = (np.asarray(fx) + 1) * CAM_W * 0.5, (np.asarray(fy) + 1) * CAM_H * 0.5
    return (px - x0) / (TILE * 0.5) - 1, (py - y0) / (TILE * 0.5) - 1


def crop(frames, t):
    """frames (..., CAM_H, CAM_W) -> tile t (..., TILE, TILE)."""
    x0, y0 = ORIGINS[t]
    return frames[..., y0:y0 + TILE, x0:x0 + TILE]


def metres_per_px(alt_m):
    return alt_m * GROUND_PER_ALT / CAM_H


def speed_limit(alt_m, sweep_s):
    """Same rule as scan_speed_limit() in scan.c: the ground moves at most half a tile per sweep."""
    return metres_per_px(alt_m) * (TILE // 2) / sweep_s if alt_m > 0 and sweep_s > 0 else 0.0


def thermal_crop(th, t):
    """th (..., CAM_H/2, CAM_W/2) thermal frame(s) -> thermal channel of tile t (..., TILE, TILE):
    every camera pixel takes the thermal pixel under it. Same as scan_thermal_tile() in scan.c."""
    x0, y0 = ORIGINS[t]
    yi, xi = (y0 + np.arange(TILE)) // THERMAL_DIV, (x0 + np.arange(TILE)) // THERMAL_DIV
    return th[..., yi[:, None], xi]
