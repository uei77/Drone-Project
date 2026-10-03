#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "proto.h"

void fc_uart_init(void);
void fc_send(const char *payload);          /* adds "$", checksum, "\n" */
void fc_heartbeat_start(void);
void fc_rx_start(void);                     /* parses $TEL / $ACK / $CMD from the FC */

/* inference task calls this after every successful model run */
void fc_model_mark_ok(float fps);

/* latest telemetry and its age; false if none received yet */
bool fc_get_telemetry(telemetry_t *out, uint32_t *age_ms);
/* newest $ACK sequence since the last call; false if none */
bool fc_take_ack(uint16_t *seq);
/* VISION_MODE_*, starts at VISION_MODE_DEFAULT, changed by $CMD,MODE */
int fc_vision_mode(void);
