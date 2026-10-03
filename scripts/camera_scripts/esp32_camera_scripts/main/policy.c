#include "policy.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

static void sendf(policy_t *p, const char *s) { if (p->send) p->send(s); }

static int pct(float c)
{
    int v = (int)lroundf(c * 100.f);
    return v < 0 ? 0 : v > 100 ? 100 : v;
}

void policy_init(policy_t *p, const sit_def_t *sit, int n, policy_send_fn send, int mode)
{
    *p = (policy_t){ .sit = sit, .n_sit = n, .send = send, .mode = mode, .cur = -1 };
}

bool policy_busy(const policy_t *p) { return p->st != POL_IDLE || p->done_left > 0; }

const char *policy_direction(const float pos[2], float db)
{
    float x = pos[0], y = pos[1];
    if (fabsf(x) < db && fabsf(y) < db) return NULL;          /* centred */
#if CAM_MOUNT == CAM_MOUNT_DOWN
    if (fabsf(x) >= fabsf(y)) return x > 0 ? "right" : "left";
    return y > 0 ? "back" : "forward";                          /* image top = nose */
#else
    if (fabsf(x) >= fabsf(y)) return x > 0 ? "right" : "left";
    return y > 0 ? "down" : "up";
#endif
}

/* end_trk: the tracker still holds this situation ACTIVE and the caller has to end it */
static void finish(policy_t *p, bool end_trk)
{
    p->done_left = 3;            /* also after an unanswered $EVT: the $ACK may be on its way */
    p->done_seq = p->seq;
    p->end_trk = end_trk;
    p->st = POL_IDLE;
    p->landing = false;
}

void policy_on_event(policy_t *p, trk_ev_t ev, uint32_t now)
{
    char b[64];
    if (ev.kind == TRK_EV_START) {
        const sit_def_t *s = &p->sit[ev.sit];
        if (s->resp == RESP_NONE || p->mode == VISION_MODE_OFF) return;
        if (p->mode == VISION_MODE_SHADOW) {
            snprintf(b, sizeof b, "SHD,%s,%d", s->name, pct(ev.conf));
            sendf(p, b);
            return;
        }
        p->cur = ev.sit;
        p->seq++;
        p->tries = 1;
        p->t_tx = now;
        p->t_seen = now;
        p->has_tgt = false;
        p->conf = ev.conf;
        p->centred = 0;
        p->landing = false;
        p->end_trk = false;
        p->st = POL_WAIT_ACK;
        snprintf(b, sizeof b, "EVT,%u,%s,%d,%u", p->seq, s->name, pct(ev.conf), s->urgency);
        sendf(p, b);
    } else if (ev.kind == TRK_EV_END && ev.sit == p->cur) {
        p->end_trk = false;                       /* the tracker has ended it already */
        if (p->st != POL_IDLE && !p->landing) finish(p, false);   /* gone: give the mission back */
    }
}

void policy_on_ack(policy_t *p, uint16_t seq, uint32_t now)
{
    if (p->st == POL_WAIT_ACK && seq == p->seq) {
        p->st = POL_RUNNING;
        p->t_run = now;
        p->t_act = now - ACT_PERIOD_MS;    /* first $ACT right away */
    }
}

void policy_observe(policy_t *p, bool has_tgt, const float pos[2], float conf, float alt_m,
                    uint32_t now)
{
    if (p->st == POL_IDLE) return;
    const sit_def_t *s = &p->sit[p->cur];
    p->conf = conf;
    p->alt_m = alt_m;
    p->has_tgt = has_tgt;
    if (!has_tgt) { p->centred = 0; return; }
    p->t_seen = now;
    p->pos[0] = pos[0];
    p->pos[1] = pos[1];
    if (policy_direction(pos, CENTER_DEADBAND)) p->centred = 0;
    else if (p->centred < CENTER_SWEEPS) p->centred++;
    if (p->st == POL_RUNNING && s->resp == RESP_CENTER_LAND && p->centred >= CENTER_SWEEPS &&
        alt_m > 0.f && alt_m <= LAND_FINAL_ALT_M)
        p->landing = true;
}

bool policy_tick(policy_t *p, uint32_t now)
{
    char b[64];
    if (p->done_left > 0) {
        snprintf(b, sizeof b, "EVT,%u,done", p->done_seq);
        sendf(p, b);
        if (--p->done_left > 0 || !p->end_trk) return false;
        p->end_trk = false;
        return true;
    }
    if (p->st == POL_IDLE) return false;
    const sit_def_t *s = &p->sit[p->cur];

    if (p->st == POL_WAIT_ACK) {
        if (now - p->t_tx < EVT_RETRY_MS) return false;
        if (p->tries >= EVT_MAX_TRIES) {         /* FC is not listening: never steer blind */
            p->st = POL_IDLE;
            return true;
        }
        p->tries++;
        p->t_tx = now;
        snprintf(b, sizeof b, "EVT,%u,%s,%d,%u", p->seq, s->name, pct(p->conf), s->urgency);
        sendf(p, b);
        return false;
    }

    /* POL_RUNNING */
    if (now - p->t_run >= RESP_MAX_MS) { finish(p, true); return false; }
    if (now - p->t_act < ACT_PERIOD_MS) return false;
    p->t_act = now;

    const char *act = "hold";
    switch (s->resp) {
    case RESP_REPORT:
        finish(p, true);
        return false;
    case RESP_HOLD_REPORT:
        if (now - p->t_run >= s->hold_ms) { finish(p, true); return false; }
        break;
    case RESP_CLIMB_HOLD:
        if (now - p->t_run >= s->hold_ms) { finish(p, true); return false; }
        act = "up";
        break;
    case RESP_CENTER_HOLD:
    case RESP_CENTER_LAND:
        if (p->landing) { act = "land"; break; }
        if (!p->has_tgt || now - p->t_seen > TGT_LOST_MS) break;            /* lost: hold */
        snprintf(b, sizeof b, "TGT,%u,%d,%d,%d", p->seq, (int)lroundf(p->pos[0] * 1000.f),
                 (int)lroundf(p->pos[1] * 1000.f), pct(p->conf));
        sendf(p, b);
        {
            const char *d = policy_direction(p->pos, CENTER_DEADBAND);
            if (d) act = d;
            else if (p->centred >= CENTER_SWEEPS) {
                /* landing marker: go down in small steps and keep correcting on the way;
                 * policy_observe() switches to "land" below LAND_FINAL_ALT_M */
                if (s->resp == RESP_CENTER_LAND) { if (p->alt_m > 0.f) act = "down"; }
                else if (now - p->t_run >= s->hold_ms) { finish(p, true); return false; }
            }
        }
        break;
    default:
        break;
    }
    snprintf(b, sizeof b, "ACT,%u,%s,%d", p->seq, act, pct(p->conf));
    sendf(p, b);
    return false;
}
