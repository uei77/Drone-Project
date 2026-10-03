#pragma once
/*
 * Tile scan over one camera frame (pure C, host-testable).
 * The frame is CAM_W x CAM_H; the model sees SCAN_TILES overlapping TILE x TILE windows.
 * A sweep collects the model output of every tile and reduces it to one answer per situation:
 * the highest probability over all tiles, which tile it came from, and where the target is
 * in the WHOLE frame.
 */
#include <stdint.h>
#include "config.h"

#define SCAN_MAX_CLASSES 16

typedef struct {
    int k;                                   /* number of classes incl. normal */
    int n;                                   /* tiles added so far */
    float p[SCAN_MAX_CLASSES];               /* [0] = lowest "normal" of any tile, others = highest */
    float pos[SCAN_MAX_CLASSES][2];          /* target in the whole frame, [-1, 1], per situation */
    int tile[SCAN_MAX_CLASSES];              /* tile that gave p[c] */
} scan_result_t;

/* top-left pixel of tile t (0 .. SCAN_TILES-1, row by row) */
void scan_tile_origin(int t, int *x0, int *y0);
/* tile-local position ([-1,1] inside the tile) -> position in the whole frame ([-1,1]) */
void scan_to_frame(int t, const float tile_pos[2], float frame_pos[2]);

/* thermal channel of tile t: every camera pixel takes the thermal pixel under it.
 * th: THERMAL_W x THERMAL_H frame, out: TILE x TILE. Same as tiles.thermal_crop() in training. */
void scan_thermal_tile(const uint8_t *th, int t, uint8_t *out);

void scan_begin(scan_result_t *r, int k);
void scan_add(scan_result_t *r, int t, const float *prob, const float tile_pos[2]);

/* Highest ground speed (m/s) at which the model still works: the ground may move at most half
 * a tile between two sweeps, so a target is seen about 5 times while it crosses the frame and
 * the previous image still overlaps the current one. 0 if altitude or sweep time is unknown.
 * Training uses the same rule (training/prepare_data.py). */
float scan_speed_limit(float alt_m, float sweep_s);
