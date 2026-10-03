#include "tracker.h"
#include <string.h>

void tracker_init(tracker_t *t, const trk_cfg_t *cfg, int n)
{
    memset(t, 0, sizeof(*t));
    t->cfg = cfg;
    t->n = n > TRK_MAX ? TRK_MAX : n;
    t->active = -1;
}

void tracker_reset(tracker_t *t)
{
    tracker_init(t, t->cfg, t->n);
}

static void enter(trk_slot_t *s, trk_phase_t ph, uint32_t now)
{
    s->phase = ph;
    s->t_phase = now;
    s->on_cnt = 0;
    s->off_cnt = 0;
    if (ph == TRK_NORMAL || ph == TRK_COOLDOWN) s->peak = 0.f;
}

trk_ev_t tracker_end(tracker_t *t, uint32_t now)
{
    trk_ev_t ev = { TRK_EV_NONE, -1, 0.f };
    if (t->active >= 0) {
        ev.kind = TRK_EV_END;
        ev.sit = t->active;
        ev.conf = t->slot[t->active].peak;
        enter(&t->slot[t->active], TRK_COOLDOWN, now);
        t->active = -1;
    }
    return ev;
}

trk_ev_t tracker_update(tracker_t *t, const float *p, bool gate, uint32_t now)
{
    trk_ev_t ev = { TRK_EV_NONE, -1, 0.f };
    int best = -1;

    for (int i = 1; i < t->n; i++) {
        const trk_cfg_t *c = &t->cfg[i];
        trk_slot_t *s = &t->slot[i];
        float pi = p[i];
        switch (s->phase) {
        case TRK_COOLDOWN:
            if (now - s->t_phase < c->cooldown_ms) break;
            enter(s, TRK_NORMAL, now);
            /* this frame can already start a new candidate */
            __attribute__((fallthrough));
        case TRK_NORMAL:
            if (gate && pi >= c->thr) {
                enter(s, TRK_CANDIDATE, now);
                s->on_cnt = 1;
                s->peak = pi;
            }
            break;
        case TRK_CANDIDATE:
            if (!gate || pi < c->thr_low) { enter(s, TRK_NORMAL, now); break; }
            if (pi >= c->thr && s->on_cnt < 255) s->on_cnt++;
            if (pi > s->peak) s->peak = pi;
            break;
        case TRK_ACTIVE:
            if (pi > s->peak) s->peak = pi;
            if (pi < c->thr_low) s->off_cnt++; else s->off_cnt = 0;
            if (s->off_cnt >= c->m_off || now - s->t_phase >= c->max_active_ms) {
                ev = tracker_end(t, now);
            }
            break;
        }
        if (s->phase == TRK_CANDIDATE && s->on_cnt >= c->n_on && c->n_on > 0) {
            if (best < 0 || c->priority > t->cfg[best].priority ||
                (c->priority == t->cfg[best].priority && pi > p[best]))
                best = i;
        }
    }

    if (ev.kind == TRK_EV_NONE && t->active < 0 && best >= 0 && gate) {
        trk_slot_t *s = &t->slot[best];
        enter(s, TRK_ACTIVE, now);
        s->peak = p[best];
        t->active = best;
        ev.kind = TRK_EV_START;
        ev.sit = best;
        ev.conf = p[best];
        /* the others restart counting: one event at a time */
        for (int i = 1; i < t->n; i++)
            if (i != best && t->slot[i].phase == TRK_CANDIDATE) enter(&t->slot[i], TRK_NORMAL, now);
    }
    return ev;
}
