#include <stdio.h>
#include "esp_err.h"
#include "fc_uart.h"
#include "camera.h"
#include "wifi.h"
#include "net.h"
void app_main(void)
{
    fc_uart_init();
    fc_heartbeat_start();
    if (camera_start() != ESP_OK) camera_recover();
    wifi_init();
    net_start();

}