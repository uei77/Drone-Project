#include "proto.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f

uint8_t proto_checksum(const char *p, size_t n)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < n; i++) cs ^= (uint8_t)p[i];
    return cs;
}

int proto_frame(const char *payload, char *out, size_t out_size)
{
    size_t n = strlen(payload);
    int k = snprintf(out, out_size, "$%s*%02X\n", payload, proto_checksum(payload, n));
    return (k > 0 && (size_t)k < out_size) ? k : -1;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* splits payload in place on ',' ; returns field count */
static int split(char *s, char *f[], int max)
{
    int n = 0;
    f[n++] = s;
    for (; *s && n < max; s++) {
        if (*s == ',') { *s = 0; f[n++] = s + 1; }
    }
    return n;
}

static bool to_f(const char *s, float *v)
{
    char *e;
    *v = strtof(s, &e);
    return e != s && *e == 0 && isfinite(*v);
}

static bool to_l(const char *s, long *v)
{
    char *e;
    *v = strtol(s, &e, 10);
    return e != s && *e == 0;
}

msg_t proto_parse(const char *line)
{
    msg_t m = { .kind = MSG_BAD };
    char buf[FC_LINE_MAX + 1];
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\n' || line[n - 1] == '\r')) n--;
    if (n < 5 || n > FC_LINE_MAX || line[0] != '$' || line[n - 3] != '*') return m;
    int h1 = hexval(line[n - 2]), h2 = hexval(line[n - 1]);
    if (h1 < 0 || h2 < 0) return m;
    size_t plen = n - 4;
    if (proto_checksum(line + 1, plen) != (uint8_t)(h1 * 16 + h2)) return m;
    memcpy(buf, line + 1, plen);
    buf[plen] = 0;

    char *f[16];
    int nf = split(buf, f, 16);
    if (strcmp(f[0], "TEL") == 0 && nf == 12) {
        telemetry_t *t = &m.tel;
        long ms, wi, wt, md;
        if (!to_l(f[1], &ms) || !to_f(f[2], &t->alt_m) || !to_f(f[3], &t->hdg_deg) ||
            !to_f(f[4], &t->vz_mps) || !to_f(f[5], &t->vn_mps) || !to_f(f[6], &t->ve_mps) ||
            !to_f(f[7], &t->yawrate_dps) || !to_l(f[8], &wi) || !to_l(f[9], &wt) ||
            !to_f(f[10], &t->wp_brg_deg) || !to_l(f[11], &md) ||
            wi < 0 || wt < 0 || wi > 65535 || wt > 65535 || md < 0 || md > 3)
            return m;
        t->t_ms = (uint32_t)ms;
        t->wp_idx = (uint16_t)wi;
        t->wp_total = (uint16_t)wt;
        t->mode = (fc_mode_t)md;
        m.kind = MSG_TEL;
    } else if (strcmp(f[0], "ACK") == 0 && nf == 2) {
        long s;
        if (!to_l(f[1], &s) || s < 0 || s > 65535) return m;
        m.seq = (uint16_t)s;
        m.kind = MSG_ACK;
    } else if (strcmp(f[0], "CMD") == 0 && nf == 3 && strcmp(f[1], "MODE") == 0) {
        long v;
        if (!to_l(f[2], &v) || v < 0 || v > 2) return m;
        m.mode = (int)v;
        m.kind = MSG_CMD_MODE;
    }
    return m;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

void telemetry_features(const telemetry_t *t, float dt_s, float out[TEL_FEATURES])
{
    memset(out, 0, sizeof(float) * TEL_FEATURES);
    float h = t->hdg_deg * DEG2RAD;
    float ch = cosf(h), sh = sinf(h);
    float alt = t->alt_m < 0.5f ? 0.5f : t->alt_m;
    float gs = sqrtf(t->vn_mps * t->vn_mps + t->ve_mps * t->ve_mps);
    float v_fwd = t->vn_mps * ch + t->ve_mps * sh;     /* body frame */
    float v_rgt = -t->vn_mps * sh + t->ve_mps * ch;
    float rel = (t->wp_brg_deg - t->hdg_deg) * DEG2RAD;

    out[0] = clampf(t->alt_m / TEL_ALT_SCALE_M, 0.f, 1.f);
    out[1] = sh;
    out[2] = ch;
    out[3] = clampf(t->vz_mps / TEL_VZ_SCALE_MPS, -1.f, 1.f);
    out[4] = clampf(gs / TEL_GS_SCALE_MPS, 0.f, 1.f);
    out[5] = t->wp_total ? clampf((float)t->wp_idx / t->wp_total, 0.f, 1.f) : 0.f;
    out[6] = sinf(rel);
    out[7] = cosf(rel);
    out[8] = clampf(dt_s / TEL_DT_SCALE_S, 0.f, 1.f);
    out[9] = clampf(v_fwd / alt / TEL_EGO_SCALE, -1.f, 1.f);
    out[10] = clampf(v_rgt / alt / TEL_EGO_SCALE, -1.f, 1.f);
    out[11] = clampf(t->yawrate_dps / TEL_YAWRATE_SCALE, -1.f, 1.f);
}
