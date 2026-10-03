#pragma once
/*
 * Thermal camera. One frame per sweep, already brought onto the camera grid:
 * THERMAL_W x THERMAL_H bytes, 128 = median temperature of the frame, 8 counts per degree
 * (config.h). The sensor is not chosen yet, so thermal.c has no driver: thermal_init()
 * reports "no sensor" and the model runs on the camera alone.
 */
#include <stdbool.h>
#include <stdint.h>

bool thermal_init(void);                 /* true if a sensor answered */
bool thermal_read(uint8_t *frame);       /* true if a fresh frame was written */
