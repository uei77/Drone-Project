#pragma once
/*
 * Situation -> response. The model only says WHAT it sees; this table says WHAT TO DO.
 * Changing a response or a threshold needs no retraining (pure C, host-testable).
 */
#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

typedef enum {
    RESP_NONE = 0,      /* never reported (e.g. "normal") */
    RESP_REPORT,        /* $EVT only, FC decides; done right after ACK */
    RESP_HOLD_REPORT,   /* hold position for hold_ms, then done */
    RESP_CENTER_HOLD,   /* steer to centre the target, then hold for hold_ms */
    RESP_CENTER_LAND,   /* centre, descend while staying centred, land below LAND_FINAL_ALT_M */
    RESP_CLIMB_HOLD,    /* climb for hold_ms (get away from an approaching object) */
} resp_t;

typedef struct {
    const char *name;   /* must match the model's class order (model_meta.h) */
    trk_cfg_t trk;
    resp_t resp;
    uint8_t urgency;    /* 0 low, 1 medium, 2 high: a table property now, not a model head */
    uint32_t hold_ms;
} sit_def_t;

typedef void (*policy_send_fn)(const char *payload);

typedef enum { POL_IDLE = 0, POL_WAIT_ACK, POL_RUNNING } pol_state_t;

typedef struct {
    const sit_def_t *sit;
    int n_sit;
    policy_send_fn send;
    int mode;               /* VISION_MODE_* */
    pol_state_t st;
    int cur;                /* situation being handled */
    uint16_t seq;
    int tries;
    uint32_t t_tx, t_run, t_act, t_seen;
    /* last observation (one per sweep) */
    bool has_tgt;
    float pos[2], conf, alt_m;
    int centred;            /* sweeps in a row with the target inside the deadband */
    bool landing;
    int done_left;          /* remaining $EVT,done repeats */
    uint16_t done_seq;      /* sequence number those repeats belong to */
    bool end_trk;           /* the response ended by itself: caller must end the tracker */
} policy_t;

void policy_init(policy_t *p, const sit_def_t *sit, int n_sit, policy_send_fn send, int mode);
/* tracker START/END */
void policy_on_event(policy_t *p, trk_ev_t ev, uint32_t now);
void policy_on_ack(policy_t *p, uint16_t seq, uint32_t now);
/* once per sweep: what the model saw. alt_m <= 0 means "altitude unknown" (no descent). */
void policy_observe(policy_t *p, bool has_tgt, const float pos[2], float conf, float alt_m,
                    uint32_t now);
/* as often as possible (after every tile): retries, $ACT / $TGT, timeouts.
 * Returns true when the response is over and the caller must call tracker_end(). */
bool policy_tick(policy_t *p, uint32_t now);
/* image offset -> one of the allowed actions (depends on CAM_MOUNT) */
const char *policy_direction(const float pos[2], float deadband);
bool policy_busy(const policy_t *p);
