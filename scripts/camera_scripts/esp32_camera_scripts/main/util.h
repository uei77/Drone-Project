#pragma once
#include <stdint.h>
#include "esp_timer.h"

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}
