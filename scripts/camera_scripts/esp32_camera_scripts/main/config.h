#pragma once
/*
 * DroneLaya v2.2 firmware settings (tiled scan: 320x240 frame, twelve 96x96 tiles).
 * Pure C header (no ESP-IDF includes) so the host unit tests can use it too.
 * now_ms() moved to util.h. Situation list and per-situation policy live in situations.c.
 */
#include <stdint.h>

/* ---------- Data-collection build (Wi-Fi). 0 = flight build: Wi-Fi never starts ---------- */
#define COLLECT_MODE     0
/* Your real Wi-Fi name, password and laptop address go into main/wifi_secret.h (same three
 * #defines). That file is in .gitignore, so the password never reaches GitHub. */
#if defined(__has_include) && __has_include("wifi_secret.h")
#include "wifi_secret.h"
#else
#define WIFI_SSID        "connected-hotspot"
#define WIFI_PASS        "hotspot-password"
#define COLLECT_HOST_IP  "192.168.137.1"   /* laptop running training/recorder.py */
#endif
#define COLLECT_FRAME_MS 20                /* pause after each frame; ~10 fps of 320x240 in total */

/* ---------- UART to the flight controller (pins are in board.h) ---------- */
#define FC_UART          UART_NUM_1
#define FC_BAUD          115200
#define FC_LINE_MAX      96          /* longest protocol line incl. "$" and "*HH\n" */

/* ---------- Camera frame and tile scan ----------
 * From 25-30 m a person is ~1.5 px wide in a 96x96 view of the whole scene: invisible.
 * So the camera gives 320x240 (~10 cm per pixel at 28 m) and the model looks at twelve
 * overlapping 96x96 tiles, one after the other. One pass over all tiles = one "sweep".
 * The tracker and the policy work per sweep. Same numbers in training/tiles.py. */
#define CAM_W            320
#define CAM_H            240
#define TILE             96          /* model input size */
#define SCAN_COLS        4
#define SCAN_ROWS        3
#define SCAN_TILES       (SCAN_COLS * SCAN_ROWS)
#define CAM_FAIL_REINIT  5           /* consecutive failed grabs before camera_recover() */

/* ---------- Thermal frame (third image channel of a v22 model) ----------
 * The thermal driver hands over one frame per sweep that is already on the camera grid at
 * 1/THERMAL_DIV of its resolution: 128 = median temperature of the frame, 8 counts per degree.
 * Whatever the sensor's own resolution, the driver scales and shifts it to this size.
 * No driver yet (thermal.c): the channel is empty and telemetry slot 12 says so. */
#define THERMAL_DIV      2
#define THERMAL_W        (CAM_W / THERMAL_DIV)
#define THERMAL_H        (CAM_H / THERMAL_DIV)

/* ---------- Timing and limits ---------- */
#define HEARTBEAT_MS         200     /* $HB period */
#define MODEL_ALIVE_MS       2000    /* $HB says OK only if the model ran within this window */
#define TEL_STALE_MS         500     /* no decision when telemetry is older than this */
#define FRAME_GAP_RESET_MS   3000    /* longer gap between sweeps -> reset memory (h, prev frame) */
#define EVT_RETRY_MS         200     /* resend $EVT until $ACK */
#define EVT_MAX_TRIES        5       /* no $ACK after this -> drop the event, no $ACT sent */
#define ACT_PERIOD_MS        250     /* $ACT rate while a response runs (FC holds after 1 s silence) */
#define RESP_MAX_MS          120000  /* hard limit for any response, then $EVT,done (a guided descent needs ~1 min) */
#define TGT_LOST_MS          2500    /* target not seen for this long -> hold (must be longer than one sweep) */

/* ---------- Camera mounting: how image offset maps to drone motion ---------- */
#define CAM_MOUNT_DOWN       0       /* looking straight down, image top = drone nose */
#define CAM_MOUNT_FORWARD    1       /* looking forward */
#define CAM_MOUNT            CAM_MOUNT_DOWN
#define CENTER_DEADBAND      0.20f   /* |x| and |y| below this count as centred */
#define CENTER_SWEEPS        3       /* centred this many sweeps in a row before going down / ending the hold */
#define LAND_FINAL_ALT_M     10.0f   /* centre-and-land: correct the position down to here, then "land".
                                        Lower is more precise, but the marker must still fit in one tile. */
/* length of the ground seen along the flight direction (frame height) per metre of altitude.
 * 0.86 = 24 m at 28 m, a first estimate: measure it for your lens (KURULUM.md / SETUP.md). */
#define CAM_GROUND_PER_ALT   0.86f

/* ---------- Run mode (changeable at run time with $CMD,MODE,<n>) ---------- */
#define VISION_MODE_OFF      0       /* model runs, only $HB is sent */
#define VISION_MODE_SHADOW   1       /* would-be events are sent as $SHD (FC only logs them) */
#define VISION_MODE_ACTIVE   2       /* $EVT / $ACT / $TGT are sent */
#define VISION_MODE_DEFAULT  VISION_MODE_SHADOW   /* first flights are shadow only */

/* ---------- Telemetry normalisation (must match training: telemetry_features()) ---------- */
#define TEL_ALT_SCALE_M      50.0f
#define TEL_VZ_SCALE_MPS     3.0f
#define TEL_GS_SCALE_MPS     10.0f
#define TEL_DT_SCALE_S       0.30f
#define TEL_EGO_SCALE        0.50f   /* (horizontal speed / altitude) in 1/s */
#define TEL_YAWRATE_SCALE    90.0f   /* deg/s */

/* drone actions understood by the FC script */
#define ALLOWED_ACTIONS_LIST \
    "hold", "forward", "back", "left", "right", "up", "down", "land"
