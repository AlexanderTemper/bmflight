#pragma once

#include "global.h"
#include "fc/fc.h"
#include "imu/imu.h"

/**
 * attitude controller
 * @param attitudeCommand
 * @param att
 * @param rateCommand
 * @param currentTime
 */
void updateAttitudeController(attitude_command_t *attitudeCommand, attitudeEulerAngles_t *att, rate_command_t *rateCommand, timeUs_t currentTime);
