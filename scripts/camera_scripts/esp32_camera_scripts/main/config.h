#pragma once
#include <stdint.h>
#include "esp_timer.h"
/*change this parts with your device and network informations */
#define WIFI_SSID   "connected-hotspot"
#define WIFI_PASS   "hotspot-password"
#define SERVER_URL  "server-url"
#define API_TOKEN   "api-token"
#define DEVICE_ID   "device-id"
/* ---------- Card informations ---------- */
#define FC_UART          UART_NUM_1
#define FC_TX_PIN        14
#define FC_RX_PIN        15
#define FC_BAUD          115200
#define CAM_FRAME_SIZE   FRAMESIZE_VGA   // faster method: FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY 15              // 10-20 interval quality qoefficient defines file storage 
#define MAX_JPEG_BYTES   (60 * 1024)
/* Website Requests*/
#define HTTP_TIMEOUT_MS  2500 // Max limit for one request
#define MAX_AGE_MS       1500  // max limit for responding 
#define LINK_TIMEOUT_MS  2000
#define HEARTBEAT_MS     200 //heartbeat interval
#define MIN_CONF_PCT     60 //min trust rate
#define CAM_FAIL_REINIT  5 //max cam fail for re initializing
#define HTTP_FAIL_RESET  3 
#define BACKOFF_MAX_MS   1000
/* drone-actions*/
#define ALLOWED_ACTIONS_LIST \
    "hold", "forward", "back", "left", "right", "up", "down", "land"

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}