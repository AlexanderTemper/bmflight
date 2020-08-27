#pragma once

#include "global.h"
#include "fc/fc.h"
#include "sensor/sensor.h"

/**
 * calculate mixer_command based on rate_command
 * @param rateCommand
 * @param gyroDev_t
 * @param mixerCommand
 * @param currentTime
 */
void updateRateController(rate_command_t *rateCommand, gyroDev_t *gyro, mixer_command_t *mixerCommand, timeUs_t currentTime);
