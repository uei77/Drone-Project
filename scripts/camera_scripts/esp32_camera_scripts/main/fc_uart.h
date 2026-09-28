#pragma once
void fc_uart_init(void);
void fc_send(const char *payload);
void fc_heartbeat_start(void);
void fc_link_mark_ok(void);