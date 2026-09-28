#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_camera.h"
#include "mbedtls/base64.h"
#include "cJSON.h"
#include "net.h"
#include "wifi.h"
#include "camera.h"
#include "fc_uart.h"
#include "config.h"
#define MIN_U32(a, b) ((a) < (b) ? (a) : (b))
static const char *TAG = "net";
static const char *ALLOWED_ACTIONS[] = { ALLOWED_ACTIONS_LIST };
static char  *s_body = NULL;
static size_t s_body_cap = 0;
static char resp_buf[512];
static int  resp_len = 0;
static bool resp_overflow = false;
static esp_err_t http_event(esp_http_client_event_t *e)
{
    if (e->event_id == HTTP_EVENT_ON_DATA) {
        if (resp_len + e->data_len < (int)sizeof(resp_buf)) {
            memcpy(resp_buf + resp_len, e->data, e->data_len);
            resp_len += e->data_len;
        } else {
            resp_overflow = true;
        }
    }
    return ESP_OK;
}
static esp_http_client_handle_t http_create(void)
{
    esp_http_client_config_t cfg = {
        .url               = SERVER_URL,
        .method            = HTTP_METHOD_POST,
        .timeout_ms        = HTTP_TIMEOUT_MS,
        .event_handler     = http_event,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (c) {
        esp_http_client_set_header(c, "Content-Type", "application/json");
        esp_http_client_set_header(c, "X-Token", API_TOKEN);
    }
    return c;
}
static int build_body(const camera_fb_t *fb, uint32_t seq)
{
    int pre = snprintf(s_body, s_body_cap,
                       "{\"device_id\":\"%s\",\"seq\":%lu,\"image\":\"",
                       DEVICE_ID, (unsigned long)seq);
    if (pre < 0 || (size_t)pre >= s_body_cap) return -1;

    size_t olen = 0;
    int rc = mbedtls_base64_encode((unsigned char *)s_body + pre, s_body_cap - pre - 3,
                                   &olen, fb->buf, fb->len);
    if (rc != 0) return -1;
    memcpy(s_body + pre + olen, "\"}", 3);
    return pre + (int)olen + 2;
}
static bool action_allowed(const char *a)
{
    for (size_t i = 0; i < sizeof(ALLOWED_ACTIONS) / sizeof(ALLOWED_ACTIONS[0]); i++) {
        if (strcmp(a, ALLOWED_ACTIONS[i]) == 0) return true;
    }
    return false;
}
static bool handle_response(uint32_t seq, uint32_t captured_ms)
{
    if (resp_overflow || resp_len == 0) return false;
    resp_buf[resp_len] = '\0';
    cJSON *root = cJSON_Parse(resp_buf);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON");
        return false;
    }
    bool valid = false;
    cJSON *jseq  = cJSON_GetObjectItem(root, "seq");
    cJSON *jact  = cJSON_GetObjectItem(root, "action");
    cJSON *jconf = cJSON_GetObjectItem(root, "confidence");
     if (!cJSON_IsNumber(jseq) || (uint32_t)jseq->valuedouble != seq) {
        ESP_LOGW(TAG, "seq does not fit");
     } else if (!cJSON_IsString(jact) || !action_allowed(jact->valuestring)) {
        ESP_LOGW(TAG, "Unrestricted/invalid action");

    } else if (!cJSON_IsNumber(jconf)) {
        ESP_LOGW(TAG, "Lack of respond confidence ");
    }else{
        uint32_t age = now_ms() - captured_ms;
        if (age > MAX_AGE_MS) {
            ESP_LOGW(TAG, "Bad response: %lu ms", (unsigned long)age);
        } else {
            valid = true;
            int conf_pct = (int)(jconf->valuedouble * 100.0 + 0.5);
            if (conf_pct >= MIN_CONF_PCT) {
                char p[48];
                snprintf(p, sizeof(p), "ACT,%lu,%s,%d",
                         (unsigned long)seq, jact->valuestring, conf_pct);
                fc_send(p);
        
            }
        }
    }
    cJSON_Delete(root);
    return valid;
}
static void net_task(void *arg){
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    esp_http_client_handle_t client = http_create();
    uint32_t seq = 0, backoff = 0;
    int cam_fail = 0, http_fail = 0;
    for (;;) {
        esp_task_wdt_reset();
        if (!wifi_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (backoff) vTaskDelay(pdMS_TO_TICKS(backoff));
        uint32_t captured = now_ms();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            if (++cam_fail >= CAM_FAIL_REINIT) { camera_recover(); cam_fail = 0; }
            continue;
        }
        cam_fail = 0;
        int len = -1;
        if (fb->len <= MAX_JPEG_BYTES) {
            len = build_body(fb, ++seq);
        } else {
            ESP_LOGW(TAG, "Square size way to big: %u bayt", (unsigned)fb->len);

        }
        esp_camera_fb_return(fb);
        if (len < 0) continue;
        if (!client) client = http_create();
        if (!client) {
            backoff = BACKOFF_MAX_MS;
            continue;
        }
        resp_len = 0;
        resp_overflow = false;
        esp_http_client_set_post_field(client, s_body, len);
        esp_err_t err = esp_http_client_perform(client);
        int status = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;
        if (err == ESP_OK && status == 200) {
            http_fail = 0;
            backoff = 0;
            if (handle_response(seq, captured)) fc_link_mark_ok();
        } else {
            ESP_LOGW(TAG, "Request failed: %s, Status %d", esp_err_to_name(err), status);
            esp_http_client_close(client);
            if (++http_fail >= HTTP_FAIL_RESET) {
                esp_http_client_cleanup(client);
                client = NULL;
                http_fail = 0;
            }
            backoff = backoff ? MIN_U32(backoff * 2, BACKOFF_MAX_MS) : 100;
        }
    }
}
void net_start(void)
{
    s_body_cap = 4 * ((MAX_JPEG_BYTES + 2) / 3) + 128;
    s_body = heap_caps_malloc(s_body_cap, MALLOC_CAP_SPIRAM);
    if (!s_body) {
        ESP_LOGE(TAG, "PSRAM error");
        esp_restart();
    }
    xTaskCreatePinnedToCore(net_task, "net", 12288, NULL, 5, NULL, 1);
}