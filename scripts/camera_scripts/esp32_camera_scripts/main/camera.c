#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "camera.h"
#include "config.h"
static const char *TAG = "camera";
esp_err_t camera_start(void)
{
    camera_config_t c = {
    .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0,
    .pin_sccb_sda = 26, .pin_sccb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,    
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = CAM_FRAME_SIZE,
    .jpeg_quality = CAM_JPEG_QUALITY,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
    };
    return esp_camera_init(&c);  
}
void camera_recover(void)
{
    ESP_LOGW(TAG, " Restarting ");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = 0; i < 3; i++) {
        if (camera_start() == ESP_OK) return;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGE(TAG, "Cannot save  -> ESP32 restarting ");
    esp_restart();
}
