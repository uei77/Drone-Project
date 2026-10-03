#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "fc_uart.h"
#include "board.h"
#include "config.h"
#include "util.h"

static const char *TAG = "fc";
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_model_ok_ms = 0;
static volatile uint16_t s_fps10 = 0;
static telemetry_t s_tel;
static uint32_t s_tel_ms = 0;          /* local receive time, 0 = never */
static bool s_ack_new = false;
static uint16_t s_ack_seq = 0;
static volatile int s_mode = VISION_MODE_DEFAULT;
static uint32_t s_bad_lines = 0;

void fc_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = FC_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(FC_UART, 2048, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FC_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(FC_UART, FC_TX_PIN, FC_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void fc_send(const char *payload)
{
    char line[FC_LINE_MAX + 1];
    int n = proto_frame(payload, line, sizeof line);
    if (n > 0) uart_write_bytes(FC_UART, line, n);
}

void fc_model_mark_ok(float fps)
{
    s_model_ok_ms = now_ms() | 1;
    s_fps10 = (uint16_t)(fps * 10.f + 0.5f);
}

bool fc_get_telemetry(telemetry_t *out, uint32_t *age_ms)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t t = s_tel_ms;
    *out = s_tel;
    portEXIT_CRITICAL(&s_mux);
    *age_ms = t ? now_ms() - t : UINT32_MAX;
    return t != 0;
}

bool fc_take_ack(uint16_t *seq)
{
    bool r;
    portENTER_CRITICAL(&s_mux);
    r = s_ack_new;
    *seq = s_ack_seq;
    s_ack_new = false;
    portEXIT_CRITICAL(&s_mux);
    return r;
}

int fc_vision_mode(void) { return s_mode; }

static void handle_line(const char *line)
{
    msg_t m = proto_parse(line);
    switch (m.kind) {
    case MSG_TEL:
        portENTER_CRITICAL(&s_mux);
        s_tel = m.tel;
        s_tel_ms = now_ms() | 1;
        portEXIT_CRITICAL(&s_mux);
        break;
    case MSG_ACK:
        portENTER_CRITICAL(&s_mux);
        s_ack_seq = m.seq;
        s_ack_new = true;
        portEXIT_CRITICAL(&s_mux);
        break;
    case MSG_CMD_MODE:
        s_mode = m.mode;
        ESP_LOGI(TAG, "vision mode -> %d", m.mode);
        break;
    default:
        if (++s_bad_lines % 50 == 1) ESP_LOGW(TAG, "bad lines: %lu", (unsigned long)s_bad_lines);
        break;
    }
}

static void rx_task(void *arg)
{
    char line[FC_LINE_MAX + 2];
    int n = 0;
    bool overflow = false;
    uint8_t buf[128];
    for (;;) {
        int got = uart_read_bytes(FC_UART, buf, sizeof buf, pdMS_TO_TICKS(50));
        for (int i = 0; i < got; i++) {
            char c = (char)buf[i];
            if (c == '$') { n = 0; overflow = false; }
            if (c == '\n') {
                if (!overflow && n > 0) { line[n] = 0; handle_line(line); }
                n = 0; overflow = false;
                continue;
            }
            if (n < FC_LINE_MAX) line[n++] = c; else overflow = true;
        }
    }
}

void fc_rx_start(void)
{
    xTaskCreatePinnedToCore(rx_task, "fc_rx", 4096, NULL, 9, NULL, 0);
}

/* OK    : model ran within MODEL_ALIVE_MS and telemetry is fresh -> reactions possible
 * NOTEL : model runs but telemetry is stale -> no decisions are made
 * LOST  : model not running. FC must NOT stop the mission for this; it only ignores vision. */
static void heartbeat_task(void *arg)
{
    uint32_t hb = 0;
    for (;;) {
        uint32_t now = now_ms(), last = s_model_ok_ms;
        bool model_ok = last != 0 && now - last < MODEL_ALIVE_MS;
        portENTER_CRITICAL(&s_mux);
        bool tel_ok = s_tel_ms != 0 && now - s_tel_ms < TEL_STALE_MS;
        portEXIT_CRITICAL(&s_mux);
        char p[40];
        snprintf(p, sizeof p, "HB,%lu,%s,%u", (unsigned long)hb++,
                 !model_ok ? "LOST" : tel_ok ? "OK" : "NOTEL", model_ok ? s_fps10 : 0);
        fc_send(p);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

void fc_heartbeat_start(void)
{
    xTaskCreatePinnedToCore(heartbeat_task, "hb", 3072, NULL, 10, NULL, 0);
}
