#include "collect.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "camera.h"
#include "config.h"
#include "fc_uart.h"
#include "util.h"
#include "wifi.h"

_Static_assert((CAM_W * CAM_H) % COLLECT_SLICE == 0 && (CAM_W * CAM_H) / COLLECT_SLICE <= 255,
               "frame must split into at most 255 equal packets");

static const char *TAG = "collect";

typedef struct __attribute__((packed)) {
    uint32_t magic, frame_id;
    uint8_t chunk, n_chunks;
    uint16_t len, width, height;
    uint32_t esp_ms, tel_age_ms;
    telemetry_t tel;
} pkt_hdr_t;

void collect_run(void)
{
    wifi_init();   /* blocks until connected */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dst = { .sin_family = AF_INET, .sin_port = htons(COLLECT_PORT) };
    inet_pton(AF_INET, COLLECT_HOST_IP, &dst.sin_addr);
    ESP_LOGI(TAG, "streaming to %s:%d", COLLECT_HOST_IP, COLLECT_PORT);

    static uint8_t pkt[sizeof(pkt_hdr_t) + COLLECT_SLICE];
    const int n_chunks = CAM_W * CAM_H / COLLECT_SLICE;
    uint32_t id = 0, sent = 0, t0 = now_ms();
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { camera_recover(); continue; }
        if (fb->len != CAM_W * CAM_H) { esp_camera_fb_return(fb); continue; }
        pkt_hdr_t h = { .magic = COLLECT_MAGIC, .frame_id = id++, .n_chunks = (uint8_t)n_chunks,
                        .len = COLLECT_SLICE, .width = CAM_W, .height = CAM_H, .esp_ms = now_ms() };
        telemetry_t tel;
        uint32_t age;
        if (!fc_get_telemetry(&tel, &age)) age = UINT32_MAX;   /* locals: header is packed */
        h.tel = tel;
        h.tel_age_ms = age;
        for (int c = 0; c < n_chunks; c++) {
            h.chunk = (uint8_t)c;
            memcpy(pkt, &h, sizeof h);
            memcpy(pkt + sizeof h, fb->buf + c * COLLECT_SLICE, COLLECT_SLICE);
            sendto(sock, pkt, sizeof h + COLLECT_SLICE, 0, (struct sockaddr *)&dst, sizeof dst);
            if ((c & 7) == 7) vTaskDelay(1);     /* let the Wi-Fi stack drain: fewer lost packets */
        }
        esp_camera_fb_return(fb);
        /* no fc_model_mark_ok(): $HB stays LOST, so the FC ignores vision during collection */
        if (++sent % 50 == 0) {
            ESP_LOGI(TAG, "%lu frames, %.1f fps", (unsigned long)sent,
                     50000.f / (float)(now_ms() - t0));
            t0 = now_ms();
        }
        /* record faster than the flight rate; training subsamples frames to the real dt pattern */
        vTaskDelay(pdMS_TO_TICKS(COLLECT_FRAME_MS));
    }
}
