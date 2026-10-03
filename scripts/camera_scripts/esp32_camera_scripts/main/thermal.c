#include "thermal.h"

/* No sensor driver yet. When the sensor is chosen, this file reads it and writes the frame
 * in the format described in thermal.h (scaled and shifted onto the camera grid). */
bool thermal_init(void) { return false; }

bool thermal_read(uint8_t *frame)
{
    (void)frame;
    return false;
}
