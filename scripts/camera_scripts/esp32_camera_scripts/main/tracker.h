#pragma once
/*
 * Situation tracker: NORMAL -> CANDIDATE -> ACTIVE -> COOLDOWN (pure C, host-testable).
 *
 * One slot per situation (index 0 = "normal" is never tracked).
 *  - CANDIDATE: p >= thr. Counter +1 per frame at/above thr; a frame between thr_low and thr
 *    keeps the counter (hysteresis); below thr_low resets to NORMAL.
 *  - ACTIVE when the counter reaches n_on and no other situation is ACTIVE.
 *    Only one situation is ACTIVE at a time; the higher priority wins a same-frame tie.
 *  - ACTIVE ends after m_off frames below thr_low, or tracker_end() (response finished),
 *    or max_active_ms. Then COOLDOWN for cooldown_ms (same situation ignored).
 *  - gate == false (stale telemetry, FC not in AUTO, vision off) clears all CANDIDATEs and
 *    blocks new activations; an ACTIVE one keeps running (the response decides).
 */
#include <stdbool.h>
#include <stdint.h>

#define TRK_MAX 16

typedef struct {
    float thr;            /* calibrated probability to count a frame */
    float thr_low;        /* hysteresis floor (<= thr) */
    uint8_t n_on;         /* frames to confirm */
    uint8_t m_off;        /* frames below thr_low to end ACTIVE */
    uint8_t priority;     /* higher wins ties */
    uint32_t cooldown_ms;
    uint32_t max_active_ms;
} trk_cfg_t;

typedef enum { TRK_NORMAL = 0, TRK_CANDIDATE, TRK_ACTIVE, TRK_COOLDOWN } trk_phase_t;

typedef struct {
    trk_phase_t phase;
    uint8_t on_cnt, off_cnt;
    uint32_t t_phase;     /* when the current phase started */
    float peak;           /* highest p while CANDIDATE/ACTIVE */
} trk_slot_t;

typedef struct {
    const trk_cfg_t *cfg; /* cfg[0] unused (normal) */
    int n;                /* number of classes incl. normal */
    trk_slot_t slot[TRK_MAX];
    int active;           /* situation index or -1 */
} tracker_t;

typedef enum { TRK_EV_NONE = 0, TRK_EV_START, TRK_EV_END } trk_ev_kind_t;
typedef struct { trk_ev_kind_t kind; int sit; float conf; } trk_ev_t;

void tracker_init(tracker_t *t, const trk_cfg_t *cfg, int n_classes);
trk_ev_t tracker_update(tracker_t *t, const float *p, bool gate, uint32_t now_ms);
/* response finished or aborted: end ACTIVE now, start its cooldown */
trk_ev_t tracker_end(tracker_t *t, uint32_t now_ms);
void tracker_reset(tracker_t *t);  /* everything back to NORMAL (e.g. new mission) */
