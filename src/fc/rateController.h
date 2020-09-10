#pragma once

#include "global.h"
#include "fc/fc.h"
#include "sensor/sensor.h"

/**
 * calculate mixer_command based on rate_command
 * @param fcControl
 * @param gyro
 * @param pidConfig
 * @param currentTime
 */
void updateRateController(control_t* fcControl, gyroDev_t *gyro, pid_config_t *pidConfig, timeUs_t currentTime);


void resetRateController(void);
