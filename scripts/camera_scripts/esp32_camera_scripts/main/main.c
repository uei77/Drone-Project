#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "board.h"
#include "camera.h"
#include "collect.h"
#include "config.h"
#include "fc_uart.h"
#include "infer.h"
#include "policy.h"
#include "scan.h"
#include "situations.h"
#include "thermal.h"
#include "tracker.h"
#include "util.h"

static const char *TAG = "main";
static trk_cfg_t s_trk_cfg[MODEL_NUM_CLASSES];

static tracker_t s_trk;
static policy_t s_pol;

/* The part of the policy that must not wait for a whole sweep: $ACK, $EVT retries, $ACT.
 * Called after every tile, so its timing does not depend on how slow the board is. */
static void policy_service(void)
{
    uint32_t now = now_ms();
    uint16_t ack;
    if (fc_take_ack(&ack)) policy_on_ack(&s_pol, ack, now);
    if (policy_tick(&s_pol, now)) {
        trk_ev_t e = tracker_end(&s_trk, now);
        if (e.kind != TRK_EV_NONE) policy_on_event(&s_pol, e, now);
    }
}

/* One sweep = one camera frame, every tile through the model, one tracker/policy decision. */
static void vision_task(void *arg)
{
    for (int i = 0; i < MODEL_NUM_CLASSES; i++) s_trk_cfg[i] = SITUATIONS[i].trk;
    tracker_init(&s_trk, s_trk_cfg, MODEL_NUM_CLASSES);
    policy_init(&s_pol, SITUATIONS, MODEL_NUM_CLASSES, fc_send, fc_vision_mode());

    /* previous frame in PSRAM (76.8 KB), one recurrent state per tile */
    uint8_t *prev = heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_SPIRAM);
    static int8_t h[SCAN_TILES][MODEL_STATE_DIM];
    if (!prev) { ESP_LOGE(TAG, "no PSRAM for the previous frame: vision off"); vTaskDelete(NULL); }
    /* thermal: only for a model with a thermal channel and a sensor that answers */
    static uint8_t th_frame[THERMAL_W * THERMAL_H], th_tile[TILE * TILE];
    bool th_on = infer_uses_thermal() && thermal_init();
    ESP_LOGI(TAG, "thermal: %s", th_on ? "on" : infer_uses_thermal() ? "model wants it, no sensor" : "not used by this model");

    uint32_t last_sweep = 0, stat_t0 = now_ms(), stat_n = 0, stat_us = 0, stat_sweeps = 0;
    int cam_fail = 0;
    float fps = 0.f, sweep_ms = 0.f;
    scan_result_t res;
    infer_out_t out;

    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb || fb->len != CAM_W * CAM_H) {
            if (fb) esp_camera_fb_return(fb);
            if (++cam_fail >= CAM_FAIL_REINIT) { camera_recover(); cam_fail = 0; }
            continue;
        }
        cam_fail = 0;
        uint32_t now = now_ms();
        bool gap = last_sweep == 0 || now - last_sweep > FRAME_GAP_RESET_MS;
        if (gap) for (int t = 0; t < SCAN_TILES; t++) infer_h_reset(h[t]);
        /* after a reset there is no real gap: use a typical one (the model never saw dt = 0) */
        float dt_s = gap ? 0.2f : (now - last_sweep) / 1000.f;
        last_sweep = now;

        telemetry_t tel;
        uint32_t tel_age;
        bool have_tel = fc_get_telemetry(&tel, &tel_age);
        if (!have_tel) memset(&tel, 0, sizeof tel);
        float feat[TEL_FEATURES];
        telemetry_features(&tel, dt_s, feat);
        bool th_ok = th_on && thermal_read(th_frame);
        feat[12] = th_ok ? 1.f : 0.f;

        scan_begin(&res, MODEL_NUM_CLASSES);
        bool ok = true;
        for (int t = 0; t < SCAN_TILES && ok; t++) {
            int x0, y0;
            scan_tile_origin(t, &x0, &y0);
            const uint8_t *cur = fb->buf + y0 * CAM_W + x0;
            if (th_ok) scan_thermal_tile(th_frame, t, th_tile);
            ok = infer_run(cur, gap ? NULL : prev + y0 * CAM_W + x0, CAM_W, th_ok ? th_tile : NULL,
                           feat, h[t], &out) == ESP_OK;
            if (!ok) break;
            scan_add(&res, t, out.prob, out.pos);
            fc_model_mark_ok(fps);          /* per tile: a slow sweep must not turn $HB to LOST */
            stat_n++;
            stat_us += out.invoke_us;
            policy_service();
        }
        memcpy(prev, fb->buf, CAM_W * CAM_H);
        esp_camera_fb_return(fb);
        if (!ok) { ESP_LOGE(TAG, "inference failed"); continue; }

        /* decisions only with fresh telemetry, while the FC flies the mission
         * (or while our own response is running) */
        now = now_ms();
        s_pol.mode = fc_vision_mode();
        have_tel = fc_get_telemetry(&tel, &tel_age);      /* again: the sweep took a while */
        bool tel_fresh = have_tel && tel_age < TEL_STALE_MS;
        bool gate = tel_fresh && s_pol.mode != VISION_MODE_OFF &&
                    (tel.mode == FC_MODE_AUTO || policy_busy(&s_pol));

        trk_ev_t ev = tracker_update(&s_trk, res.p, gate, now);
        if (ev.kind != TRK_EV_NONE) {
            ESP_LOGI(TAG, "%s %s p=%.2f tile %d", ev.kind == TRK_EV_START ? "START" : "END",
                     SITUATIONS[ev.sit].name, ev.conf, res.tile[ev.sit]);
            policy_on_event(&s_pol, ev, now);
        }
        int a = s_trk.active;
        static const float no_pos[2] = { 0.f, 0.f };
        bool has_tgt = a >= 0 && res.p[a] >= SITUATIONS[a].trk.thr_low;
        policy_observe(&s_pol, has_tgt, a >= 0 ? res.pos[a] : no_pos, a >= 0 ? res.p[a] : 0.f,
                       tel_fresh ? tel.alt_m : -1.f, now);
        policy_service();

        stat_sweeps++;
        if (now - stat_t0 >= 5000) {
            fps = stat_sweeps * 1000.f / (now - stat_t0);        /* sweeps per second */
            sweep_ms = (float)(now - stat_t0) / stat_sweeps;
            int best = 1;
            for (int c = 2; c < MODEL_NUM_CLASSES; c++) if (res.p[c] > res.p[best]) best = c;
            ESP_LOGI(TAG, "sweep %.0f ms (%.1f/s), invoke %lu us avg, best=%s p=%.2f tile %d, tel age %lu ms",
                     sweep_ms, fps, (unsigned long)(stat_us / stat_n), MODEL_CLASS_NAMES[best],
                     res.p[best], res.tile[best], (unsigned long)tel_age);
            if (tel_fresh) {
                float vmax = scan_speed_limit(tel.alt_m, sweep_ms / 1000.f);
                float v = sqrtf(tel.vn_mps * tel.vn_mps + tel.ve_mps * tel.ve_mps);
                ESP_LOG_LEVEL(v > vmax ? ESP_LOG_WARN : ESP_LOG_INFO, TAG,
                              "speed %.1f m/s, limit %.1f m/s at %.0f m%s", v, vmax, tel.alt_m,
                              v > vmax ? ": TOO FAST for this sweep time" : "");
            }
            stat_t0 = now; stat_n = 0; stat_us = 0; stat_sweeps = 0;
        }
        vTaskDelay(1);   /* once per sweep: lets the idle task feed the watchdog */
    }
}

void app_main(void)
{
    fc_uart_init();
    fc_rx_start();
    fc_heartbeat_start();      /* says LOST until the model runs: FC ignores vision, mission goes on */

    for (int i = 0; i < MODEL_NUM_CLASSES; i++) {
        if (strcmp(SITUATIONS[i].name, MODEL_CLASS_NAMES[i]) != 0) {
            ESP_LOGE(TAG, "situations.c row %d '%s' != model class '%s'", i,
                     SITUATIONS[i].name, MODEL_CLASS_NAMES[i]);
            return;            /* vision stays off, HB stays LOST */
        }
    }
    ESP_LOGI(TAG, "board: %s, model: %s", BOARD_NAME, MODEL_VERSION);
    if (camera_start() != ESP_OK) camera_recover();
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        ESP_LOGI(TAG, "Camera OK: %ux%u, %u bytes",
                 (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len);
        esp_camera_fb_return(fb);
    } else {
        ESP_LOGE(TAG, "Camera could not capture a frame");
        camera_recover();
    }
#if COLLECT_MODE
    collect_run();             /* data-collection build: never returns */
#endif
    if (infer_init() != ESP_OK) {
        ESP_LOGE(TAG, "model init failed: vision off");
        return;
    }
    xTaskCreatePinnedToCore(vision_task, "vision", 10240, NULL, 5, NULL, 1);
}
