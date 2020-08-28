#pragma once

#include "global.h"
#include "sensor/sensor.h"

typedef struct {
    void (*update)(timeUs_t currentTime);
} attitude_estimator_t;

#define EULER_INITIALIZE  { { 0, 0, 0 } }

typedef union attitudeEulerAngles_s{
    int16_t raw[3];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;

extern attitudeEulerAngles_t attitude;

void initImu(void);
void updateEstimatedAttitude(timeUs_t currentTime);
