#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "fc_uart.h"
#include "camera.h"
void app_main(void)
{
    fc_uart_init();
    fc_heartbeat_start();
    if (camera_start() != ESP_OK) camera_recover();

    /* Camera self-test: grab one frame and log its size. */
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        ESP_LOGI("main", "Camera OK: %ux%u, %u bytes",
                 (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len);
        esp_camera_fb_return(fb);
    } else {
        ESP_LOGE("main", "Camera could not capture a frame");
        camera_recover();
    }

}