#pragma once
#include <stdint.h>
#include "esp_timer.h"
/*change this parts with your device and network informations */
#define WIFI_SSID   "connected-hotspot"
#define WIFI_PASS   "hotspot-password"
/* ---------- Card informations ---------- */
#define FC_UART          UART_NUM_1
#define FC_TX_PIN        14
#define FC_RX_PIN        15
#define FC_BAUD          115200
#define CAM_FRAME_SIZE   FRAMESIZE_VGA   // faster method: FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY 15              // 10-20 interval quality qoefficient defines file storage 
/* Timing and limits */
#define LINK_TIMEOUT_MS  2000
#define HEARTBEAT_MS     200 //heartbeat interval
#define MIN_CONF_PCT     60 //min trust rate
#define CAM_FAIL_REINIT  5 //max cam fail for re initializing
/* drone-actions*/
#define ALLOWED_ACTIONS_LIST \
    "hold", "forward", "back", "left", "right", "up", "down", "land"

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}