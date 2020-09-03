#pragma once

#include "global.h"
#include "fc/fc.h"
#include "imu/imu.h"

void updateAttitudeController(control_t* fcControl, bool armed, timeUs_t currentTime, attitudeEulerAngles_t *att);
