#pragma once

#include "global.h"
#include "fc/fc.h"
#include "sensor/sensor.h"

/**
 * calculate mixer_command based on rate_command
 * @param fcControl
 * @param armed
 * @param gyro
 * @param currentTime
 */
void updateRateController(control_t* fcControl, bool armed, gyroDev_t *gyro, timeUs_t currentTime);
