#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "fc_uart.h"
#include "config.h"
static volatile uint32_t s_last_valid_ms = 0;
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
    ESP_ERROR_CHECK(uart_driver_install(FC_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FC_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(FC_UART, FC_TX_PIN, FC_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}
void fc_send(const char *payload)
{
    uint8_t cs = 0;
    for (const char *p = payload; *p; p++) cs ^= (uint8_t)*p;

    char line[64];
    int n = snprintf(line, sizeof(line), "$%s*%02X\n", payload, cs);
    if (n > 0 && n < (int)sizeof(line)) {
        uart_write_bytes(FC_UART, line, n);
    }
}
void fc_link_mark_ok(void)
{
    s_last_valid_ms = now_ms() | 1;
}
static bool link_ok(void)
{
    uint32_t last = s_last_valid_ms;
    return last != 0 && (now_ms() - last) < LINK_TIMEOUT_MS;
}
static void heartbeat_task(void *arg)
{
    uint32_t hb = 0;
    for (;;) {
        char p[32];
        snprintf(p, sizeof(p), "HB,%lu,%s", (unsigned long)hb++, link_ok() ? "OK" : "LOST");
        fc_send(p);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}
void fc_heartbeat_start(void){
     xTaskCreatePinnedToCore(heartbeat_task, "hb", 3072, NULL, 10, NULL, 1);
}