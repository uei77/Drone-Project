#pragma once
/*
 * Line protocol between ESP32 and the flight controller (pure C, host-testable).
 * Every line: $<payload>*<XOR checksum of payload, 2 hex>\n   115200 8N1
 *
 * FC -> ESP32
 *   $TEL,<t_ms>,<alt_m>,<hdg_deg>,<vz_mps>,<vn_mps>,<ve_mps>,<yawrate_dps>,<wp_idx>,<wp_total>,<wp_brg_deg>,<mode>
 *        mode: 0 other, 1 AUTO (mission running), 2 PAUSED_BY_VISION, 3 RTL/LAND/failsafe
 *   $ACK,<seq>                 FC accepted $EVT <seq> (mission paused)
 *   $CMD,MODE,<0|1|2>          vision mode OFF / SHADOW / ACTIVE
 *
 * ESP32 -> FC
 *   $HB,<cnt>,<OK|NOTEL|LOST>,<fps x10>
 *   $EVT,<seq>,<situation>,<conf %>,<urgency 0-2>     repeated until $ACK
 *   $EVT,<seq>,done                                    response finished, FC resumes AUTO
 *   $ACT,<seq>,<action>,<conf %>                       only after $ACK
 *   $TGT,<seq>,<x mil>,<y mil>,<conf %>                target offset in image, -1000..1000
 *   $SHD,<situation>,<conf %>                          shadow mode: would have fired
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TEL_FEATURES 16

typedef enum { FC_MODE_OTHER = 0, FC_MODE_AUTO = 1, FC_MODE_PAUSED = 2, FC_MODE_SAFETY = 3 } fc_mode_t;

typedef struct {
    uint32_t t_ms;        /* FC timestamp (informative) */
    float alt_m;          /* above home / ground */
    float hdg_deg;
    float vz_mps;         /* up positive */
    float vn_mps, ve_mps;
    float yawrate_dps;
    uint16_t wp_idx, wp_total;
    float wp_brg_deg;     /* bearing to next waypoint, absolute */
    fc_mode_t mode;
} telemetry_t;

typedef enum { MSG_NONE = 0, MSG_TEL, MSG_ACK, MSG_CMD_MODE, MSG_BAD } msg_kind_t;

typedef struct {
    msg_kind_t kind;
    telemetry_t tel;      /* MSG_TEL */
    uint16_t seq;         /* MSG_ACK */
    int mode;             /* MSG_CMD_MODE */
} msg_t;

uint8_t proto_checksum(const char *payload, size_t n);

/* Builds "$payload*HH\n" into out. Returns length, or -1 if it does not fit. */
int proto_frame(const char *payload, char *out, size_t out_size);

/* Parses one received line (with or without trailing \r\n). Checksum is mandatory. */
msg_t proto_parse(const char *line);

/* Normalised, clamped model input. dt_s = time since previous processed frame.
 * Index map (shared with training):
 *  0 alt/50 [0,1]   1-2 sin/cos heading   3 vz/3   4 ground speed/10 [0,1]
 *  5 mission progress [0,1]   6-7 sin/cos(bearing to next WP - heading)
 *  8 dt/0.3 [0,1]   9-10 ego-motion forward/right = v/alt / 0.5   11 yaw rate/90
 *  12 thermal frame present: 0 here, set to 1 by the caller (main.c) when there is one
 *  13-15 reserved (0)                                  every value clamped to [-1,1] */
void telemetry_features(const telemetry_t *t, float dt_s, float out[TEL_FEATURES]);
