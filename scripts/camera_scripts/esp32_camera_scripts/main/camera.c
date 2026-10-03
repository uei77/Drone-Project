#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "camera.h"
#include "board.h"
#include "config.h"

_Static_assert(CAM_W == 320 && CAM_H == 240, "camera.c uses FRAMESIZE_QVGA");

static const char *TAG = "camera";

/* 320x240, 8-bit gray. The model looks at 96x96 tiles of this frame (scan.c).
 * Training data must be recorded through this same function (same size, format, settings). */
esp_err_t camera_start(void)
{
    camera_config_t c = {
        .pin_pwdn = CAM_PIN_PWDN, .pin_reset = CAM_PIN_RESET, .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size   = FRAMESIZE_QVGA,     /* must equal CAM_W x CAM_H in config.h */
        .jpeg_quality = 12,                 /* unused for grayscale */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST, /* always the newest frame, never a stale one */
    };
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) return err;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* steady exposure: big frame-to-frame brightness jumps look like motion to the model */
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, GAINCEILING_8X);
        s->set_lenc(s, 1);
    }
    return ESP_OK;
}

void camera_recover(void)
{
    ESP_LOGW(TAG, "restarting camera");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = 0; i < 3; i++) {
        if (camera_start() == ESP_OK) return;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGE(TAG, "camera cannot recover -> restarting ESP32");
    esp_restart();
}
