#pragma once

#include "global.h"
#include "sensor/sensor.h"

typedef struct {
    void (*update)(timeUs_t currentTime);
} attitude_estimator_t;

extern int16_t attitutde_r;
extern int16_t attitutde_p;
extern int16_t attitutde_y;


void initImu(attitude_estimator_t *est);
void updateEstimatedAttitude(timeUs_t currentTime);
