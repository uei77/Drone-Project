#include "scan.h"

void scan_tile_origin(int t, int *x0, int *y0)
{
    int col = t % SCAN_COLS, row = t / SCAN_COLS;
    /* spread evenly so the first tile touches the left/top edge and the last the right/bottom */
    *x0 = SCAN_COLS > 1 ? (col * (CAM_W - TILE) + (SCAN_COLS - 1) / 2) / (SCAN_COLS - 1) : 0;
    *y0 = SCAN_ROWS > 1 ? (row * (CAM_H - TILE) + (SCAN_ROWS - 1) / 2) / (SCAN_ROWS - 1) : 0;
}

void scan_to_frame(int t, const float tp[2], float fp[2])
{
    int x0, y0;
    scan_tile_origin(t, &x0, &y0);
    float px = x0 + (tp[0] + 1.f) * 0.5f * TILE;      /* pixel in the frame */
    float py = y0 + (tp[1] + 1.f) * 0.5f * TILE;
    fp[0] = px / (CAM_W * 0.5f) - 1.f;
    fp[1] = py / (CAM_H * 0.5f) - 1.f;
}

void scan_thermal_tile(const uint8_t *th, int t, uint8_t *out)
{
    int x0, y0;
    scan_tile_origin(t, &x0, &y0);
    for (int y = 0; y < TILE; y++) {
        const uint8_t *row = th + ((y0 + y) / THERMAL_DIV) * THERMAL_W;
        for (int x = 0; x < TILE; x++) *out++ = row[(x0 + x) / THERMAL_DIV];
    }
}

void scan_begin(scan_result_t *r, int k)
{
    r->k = k > SCAN_MAX_CLASSES ? SCAN_MAX_CLASSES : k;
    r->n = 0;
    for (int c = 0; c < r->k; c++) {
        r->p[c] = c == 0 ? 1.f : 0.f;
        r->pos[c][0] = r->pos[c][1] = 0.f;
        r->tile[c] = -1;
    }
}

void scan_add(scan_result_t *r, int t, const float *prob, const float tp[2])
{
    r->n++;
    if (prob[0] < r->p[0]) { r->p[0] = prob[0]; r->tile[0] = t; }
    for (int c = 1; c < r->k; c++) {
        if (prob[c] > r->p[c]) {
            r->p[c] = prob[c];
            r->tile[c] = t;
            scan_to_frame(t, tp, r->pos[c]);
        }
    }
}

float scan_speed_limit(float alt_m, float sweep_s)
{
    if (alt_m <= 0.f || sweep_s <= 0.f) return 0.f;
    float m_per_px = alt_m * CAM_GROUND_PER_ALT / CAM_H;
    return m_per_px * (TILE / 2) / sweep_s;
}
