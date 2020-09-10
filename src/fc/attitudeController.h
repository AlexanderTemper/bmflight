#pragma once

#include "global.h"
#include "fc/fc.h"
#include "imu/imu.h"

void updateAttitudeController(control_t* fcControl, config_t *fcConfig, timeUs_t currentTime, attitudeEulerAngles_t *att);
