/* Host unit tests for the pure-C firmware modules (no ESP-IDF needed).
 *   gcc -std=c11 -Wall -Wextra -Werror -I../main test_host.c ../main/proto.c \
 *       ../main/tracker.c ../main/policy.c ../main/scan.c -lm -o test_host && ./test_host
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "policy.h"
#include "proto.h"
#include "scan.h"
#include "tracker.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

/* ---------- capture of sent payloads ---------- */
static char sent[64][64];
static int n_sent = 0;
static void cap(const char *p) { if (n_sent < 64) strncpy(sent[n_sent++], p, 63); }
static int count_prefix(const char *pre)
{
    int c = 0;
    for (int i = 0; i < n_sent; i++) c += strncmp(sent[i], pre, strlen(pre)) == 0;
    return c;
}

static void test_proto(void)
{
    char line[FC_LINE_MAX + 1];
    CHECK(proto_frame("HB,0,LOST", line, sizeof line) > 0);
    CHECK(strcmp(line, "$HB,0,LOST*3E\n") == 0);      /* matches the documented example */

    char tl[128], pl[96];
    strcpy(pl, "TEL,12345,12.5,90,0.2,1.0,2.0,-15,3,10,120,1");
    CHECK(proto_frame(pl, tl, sizeof tl) > 0);
    msg_t m = proto_parse(tl);
    CHECK(m.kind == MSG_TEL);
    CHECK(fabsf(m.tel.alt_m - 12.5f) < 1e-6f && m.tel.wp_total == 10 && m.tel.mode == FC_MODE_AUTO);

    tl[5] = '9';                                       /* corrupt -> checksum fails */
    CHECK(proto_parse(tl).kind == MSG_BAD);
    CHECK(proto_parse("$ACK,7*00").kind == MSG_BAD);
    proto_frame("ACK,7", tl, sizeof tl);
    m = proto_parse(tl);
    CHECK(m.kind == MSG_ACK && m.seq == 7);
    proto_frame("CMD,MODE,2", tl, sizeof tl);
    m = proto_parse(tl);
    CHECK(m.kind == MSG_CMD_MODE && m.mode == 2);
    proto_frame("CMD,MODE,9", tl, sizeof tl);
    CHECK(proto_parse(tl).kind == MSG_BAD);
    proto_frame("TEL,1,2,3", tl, sizeof tl);          /* wrong field count */
    CHECK(proto_parse(tl).kind == MSG_BAD);
    proto_frame("TEL,1,nan,90,0,0,0,0,0,0,0,1", tl, sizeof tl);
    CHECK(proto_parse(tl).kind == MSG_BAD);

    /* features: clamped to [-1,1], heading 90 deg flying east = forward */
    telemetry_t t = { .alt_m = 200, .hdg_deg = 90, .vz_mps = -10, .vn_mps = 0, .ve_mps = 5,
                      .yawrate_dps = 30, .wp_idx = 3, .wp_total = 10, .wp_brg_deg = 90 };
    float f[TEL_FEATURES];
    telemetry_features(&t, 0.15f, f);
    for (int i = 0; i < TEL_FEATURES; i++) CHECK(f[i] >= -1.f && f[i] <= 1.f);
    CHECK(f[0] == 1.f);                               /* 200 m clamps */
    CHECK(fabsf(f[1] - 1.f) < 1e-5f && fabsf(f[2]) < 1e-5f);
    CHECK(f[3] == -1.f);
    CHECK(fabsf(f[5] - 0.3f) < 1e-6f);
    CHECK(fabsf(f[6]) < 1e-5f && fabsf(f[7] - 1.f) < 1e-5f);   /* WP straight ahead */
    CHECK(fabsf(f[8] - 0.5f) < 1e-6f);
    CHECK(f[9] > 0.f && fabsf(f[10]) < 1e-5f);       /* moving forward, not sideways */
    CHECK(f[12] == 0.f && f[15] == 0.f);
}

static const trk_cfg_t CFG[3] = {
    { 1.f, 1.f, 0, 0, 0, 0, 0 },
    { 0.8f, 0.6f, 3, 2, 1, 1000, 5000 },
    { 0.8f, 0.6f, 3, 2, 2, 1000, 5000 },
};

static void test_tracker(void)
{
    tracker_t t;
    tracker_init(&t, CFG, 3);
    float p_hi[3] = { 0.1f, 0.9f, 0.0f }, p_mid[3] = { 0.3f, 0.7f, 0.f }, p_lo[3] = { 0.9f, 0.1f, 0.f };
    uint32_t now = 0;
    CHECK(tracker_update(&t, p_hi, true, now += 100).kind == TRK_EV_NONE);
    CHECK(tracker_update(&t, p_mid, true, now += 100).kind == TRK_EV_NONE);  /* hysteresis keeps it */
    CHECK(tracker_update(&t, p_hi, true, now += 100).kind == TRK_EV_NONE);
    trk_ev_t e = tracker_update(&t, p_hi, true, now += 100);
    CHECK(e.kind == TRK_EV_START && e.sit == 1);
    CHECK(tracker_update(&t, p_lo, true, now += 100).kind == TRK_EV_NONE);
    e = tracker_update(&t, p_lo, true, now += 100);
    CHECK(e.kind == TRK_EV_END && e.sit == 1);
    /* cooldown: same situation ignored for 1000 ms */
    for (int i = 0; i < 5; i++) CHECK(tracker_update(&t, p_hi, true, now += 100).kind == TRK_EV_NONE);
    CHECK(t.slot[1].phase == TRK_COOLDOWN);
    now += 1000;
    tracker_update(&t, p_hi, true, now);
    CHECK(t.slot[1].phase == TRK_CANDIDATE);

    /* gate off: candidates cleared, nothing starts */
    tracker_init(&t, CFG, 3);
    for (int i = 0; i < 10; i++) CHECK(tracker_update(&t, p_hi, false, now += 100).kind == TRK_EV_NONE);
    CHECK(t.active == -1);

    /* a single dip below thr_low resets the count */
    tracker_init(&t, CFG, 3);
    tracker_update(&t, p_hi, true, now += 100);
    tracker_update(&t, p_hi, true, now += 100);
    tracker_update(&t, p_lo, true, now += 100);
    CHECK(tracker_update(&t, p_hi, true, now += 100).kind == TRK_EV_NONE);

    /* tie: higher priority wins, only one ACTIVE */
    tracker_init(&t, CFG, 3);
    float both[3] = { 0.f, 0.85f, 0.85f };
    tracker_update(&t, both, true, now += 100);
    tracker_update(&t, both, true, now += 100);
    e = tracker_update(&t, both, true, now += 100);
    CHECK(e.kind == TRK_EV_START && e.sit == 2);
    CHECK(t.slot[1].phase != TRK_ACTIVE);

    /* max_active_ms ends a stuck event */
    tracker_init(&t, CFG, 3);
    for (int i = 0; i < 3; i++) tracker_update(&t, p_hi, true, now += 100);
    CHECK(t.active == 1);
    e = tracker_update(&t, p_hi, true, now += 6000);
    CHECK(e.kind == TRK_EV_END);
}

static const sit_def_t SIT[3] = {
    { "normal", { 1.f, 1.f, 0, 0, 0, 0, 0 }, RESP_NONE, 0, 0 },
    { "hold",   { 0.8f, 0.6f, 3, 2, 1, 1000, 5000 }, RESP_HOLD_REPORT, 1, 1000 },
    { "marker", { 0.8f, 0.6f, 3, 2, 2, 1000, 30000 }, RESP_CENTER_LAND, 1, 0 },
};

/* one sweep: observation, then the periodic part */
static float g_alt = 8.f;
static bool step(policy_t *p, bool has, const float pos[2], float conf, uint32_t t)
{
    policy_observe(p, has, pos, conf, g_alt, t);
    return policy_tick(p, t);
}

static void test_policy(void)
{
    policy_t p;
    float pos[2] = { 0.f, 0.f };
    trk_ev_t start = { TRK_EV_START, 1, 0.9f };

    /* shadow: one $SHD, nothing else */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_SHADOW);
    policy_on_event(&p, start, 0);
    for (uint32_t t = 0; t < 3000; t += 100) step(&p, true, pos, 0.9f, t);
    CHECK(n_sent == 1 && strcmp(sent[0], "SHD,hold,90") == 0);

    /* active, FC never ACKs: EVT x5, then gives up, NO $ACT */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, start, 0);
    bool fin = false;
    for (uint32_t t = 0; t < 3000 && !fin; t += 100) fin = step(&p, true, pos, 0.9f, t);
    CHECK(fin);
    CHECK(count_prefix("EVT,1,hold") == EVT_MAX_TRIES);
    CHECK(count_prefix("ACT") == 0);

    /* active, ACK -> hold for 1 s -> done x3 */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, start, 0);
    policy_on_ack(&p, 99, 50);                       /* wrong seq ignored */
    CHECK(p.st == POL_WAIT_ACK);
    policy_on_ack(&p, 1, 100);
    fin = false;
    uint32_t t;
    for (t = 100; t < 5000 && !fin; t += 50) fin = step(&p, true, pos, 0.9f, t);
    CHECK(fin);
    CHECK(count_prefix("ACT,1,hold") >= 4);
    CHECK(count_prefix("EVT,1,done") == 3);
    CHECK(t < 1500);

    /* centre then land: target right -> "right", then centred -> "land" */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    trk_ev_t st2 = { TRK_EV_START, 2, 0.95f };
    policy_on_event(&p, st2, 0);
    policy_on_ack(&p, 1, 10);
    float right[2] = { 0.6f, 0.1f };
    step(&p, true, right, 0.95f, 20);
    CHECK(count_prefix("ACT,1,right") == 1);
    CHECK(count_prefix("TGT,1,600,100") == 1);
    for (t = 300; t < 3000; t += 250) step(&p, true, pos, 0.95f, t);
    CHECK(count_prefix("ACT,1,land") >= 1);
    /* the situation disappearing while landing must not abort the landing */
    trk_ev_t end2 = { TRK_EV_END, 2, 0.f };
    policy_on_event(&p, end2, 3000);
    CHECK(p.st == POL_RUNNING && p.landing);

    /* situation gone mid-response -> done, mission handed back */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, start, 0);
    policy_on_ack(&p, 1, 10);
    step(&p, true, pos, 0.9f, 20);
    trk_ev_t end1 = { TRK_EV_END, 1, 0.f };
    policy_on_event(&p, end1, 100);
    fin = false;
    for (t = 120; t < 1000; t += 20) fin |= step(&p, true, pos, 0.9f, t);
    CHECK(!fin && count_prefix("EVT,1,done") == 3);   /* tracker ended it: nothing left to end */

    /* guided descent: high up and centred -> "down", never "land"; off-centre -> correct first;
     * unknown altitude -> hold; low enough -> "land" */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, st2, 0);
    policy_on_ack(&p, 1, 10);
    g_alt = 28.f;
    for (t = 20; t < 2000; t += 250) step(&p, true, pos, 0.95f, t);
    CHECK(count_prefix("ACT,1,down") >= 4 && count_prefix("ACT,1,land") == 0);
    n_sent = 0;
    step(&p, true, right, 0.95f, t += 250);
    CHECK(count_prefix("ACT,1,right") == 1);
    g_alt = -1.f;
    n_sent = 0;
    for (int i = 0; i < 5; i++) step(&p, true, pos, 0.95f, t += 250);
    CHECK(count_prefix("ACT,1,hold") == 5);
    g_alt = 9.f;
    n_sent = 0;
    for (int i = 0; i < 3; i++) step(&p, true, pos, 0.95f, t += 250);
    CHECK(count_prefix("ACT,1,land") >= 1 && p.landing);
    g_alt = 8.f;

    /* ticks between two sweeps keep the last observation: target seen once, then the board
     * is busy for 1.2 s -> still steering, not "lost"; after TGT_LOST_MS -> hold */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, st2, 0);
    policy_on_ack(&p, 1, 10);
    policy_observe(&p, true, right, 0.95f, 28.f, 20);
    for (t = 20; t <= 1220; t += 100) policy_tick(&p, t);
    CHECK(count_prefix("ACT,1,right") == 5 && count_prefix("ACT,1,hold") == 0);
    for (; t <= 20 + TGT_LOST_MS + 600; t += 100) policy_tick(&p, t);
    CHECK(count_prefix("ACT,1,hold") >= 1);

    /* a second situation starts while "done" of the first is still being repeated:
     * the repeats keep the OLD sequence number and must not end the new event */
    n_sent = 0;
    policy_init(&p, SIT, 3, cap, VISION_MODE_ACTIVE);
    policy_on_event(&p, start, 0);
    policy_on_ack(&p, 1, 10);
    step(&p, true, pos, 0.9f, 20);
    policy_on_event(&p, end1, 100);
    step(&p, false, pos, 0.f, 100);
    trk_ev_t st3 = { TRK_EV_START, 2, 0.9f };
    policy_on_event(&p, st3, 200);
    fin = false;
    for (t = 200; t < 700; t += 100) fin |= step(&p, true, pos, 0.9f, t);
    CHECK(!fin);                                   /* caller must not end the tracker */
    CHECK(count_prefix("EVT,1,done") == 3 && count_prefix("EVT,2,done") == 0);
    CHECK(count_prefix("EVT,2,marker") >= 2 && p.st == POL_WAIT_ACK);

    /* direction mapping, downward camera: image top = nose */
    float up[2] = { 0.f, -0.5f }, left[2] = { -0.5f, 0.1f }, ctr[2] = { 0.1f, -0.1f };
    CHECK(strcmp(policy_direction(up, CENTER_DEADBAND), "forward") == 0);
    CHECK(strcmp(policy_direction(left, CENTER_DEADBAND), "left") == 0);
    CHECK(policy_direction(ctr, CENTER_DEADBAND) == NULL);
}

static void test_scan(void)
{
    int x0, y0, lastx = -1, lasty = -1;
    scan_tile_origin(0, &x0, &y0);
    CHECK(x0 == 0 && y0 == 0);
    scan_tile_origin(SCAN_TILES - 1, &x0, &y0);
    CHECK(x0 == CAM_W - TILE && y0 == CAM_H - TILE);        /* last tile touches the far corner */
    for (int t = 0; t < SCAN_TILES; t++) {                   /* every tile inside, neighbours overlap */
        scan_tile_origin(t, &x0, &y0);
        CHECK(x0 >= 0 && y0 >= 0 && x0 + TILE <= CAM_W && y0 + TILE <= CAM_H);
        if (t % SCAN_COLS) CHECK(x0 - lastx < TILE && x0 > lastx);
        if (t >= SCAN_COLS && t % SCAN_COLS == 0) CHECK(y0 - lasty < TILE && y0 > lasty);
        lastx = x0;
        if (t % SCAN_COLS == 0) lasty = y0;
    }
    /* tile centre -> frame coordinates */
    float c[2] = { 0.f, 0.f }, f[2];
    scan_to_frame(0, c, f);
    CHECK(fabsf(f[0] - (48.f / 160.f - 1.f)) < 1e-6f && fabsf(f[1] - (48.f / 120.f - 1.f)) < 1e-6f);
    float br[2] = { 1.f, 1.f };
    scan_to_frame(SCAN_TILES - 1, br, f);
    CHECK(fabsf(f[0] - 1.f) < 1e-6f && fabsf(f[1] - 1.f) < 1e-6f);   /* bottom-right corner */

    /* sweep: highest situation probability wins, normal = lowest, position of the winning tile */
    scan_result_t r;
    scan_begin(&r, 3);
    float p_a[3] = { 0.9f, 0.1f, 0.0f }, p_b[3] = { 0.2f, 0.7f, 0.1f }, p_c[3] = { 0.5f, 0.2f, 0.3f };
    float pos_b[2] = { 0.5f, -0.5f };
    scan_add(&r, 0, p_a, c);
    scan_add(&r, 5, p_b, pos_b);
    scan_add(&r, 11, p_c, c);
    CHECK(r.n == 3 && fabsf(r.p[0] - 0.2f) < 1e-6f);
    CHECK(fabsf(r.p[1] - 0.7f) < 1e-6f && r.tile[1] == 5);
    CHECK(fabsf(r.p[2] - 0.3f) < 1e-6f && r.tile[2] == 11);
    scan_to_frame(5, pos_b, f);
    CHECK(r.pos[1][0] == f[0] && r.pos[1][1] == f[1]);

    /* thermal channel of a tile: camera pixel (x, y) takes thermal pixel (x / 2, y / 2) */
    static uint8_t th[THERMAL_W * THERMAL_H], tile[TILE * TILE];
    for (int i = 0; i < THERMAL_W * THERMAL_H; i++) th[i] = (uint8_t)((i % THERMAL_W) + 3 * (i / THERMAL_W));
    scan_thermal_tile(th, 5, tile);                          /* tile 5 starts at camera (75, 72) */
    CHECK(tile[0] == (uint8_t)(37 + 3 * 36) && tile[1] == (uint8_t)(38 + 3 * 36) && tile[2] == tile[1]);
    CHECK(tile[TILE * TILE - 1] == (uint8_t)((75 + 95) / 2 + 3 * ((72 + 95) / 2)));

    /* 28 m: 24.08 m of ground over 240 px -> half a tile (48 px) = 4.8 m per sweep */
    CHECK(fabsf(scan_speed_limit(28.f, 0.2f) - 24.08f) < 0.01f);
    CHECK(fabsf(scan_speed_limit(28.f, 1.2f) - 4.013f) < 0.01f);
    CHECK(scan_speed_limit(0.f, 0.2f) == 0.f && scan_speed_limit(28.f, 0.f) == 0.f);
}

int main(void)
{
    test_proto();
    test_tracker();
    test_policy();
    test_scan();
    printf("%d checks, %d failed\n", checks, fails);
    return fails != 0;
}
