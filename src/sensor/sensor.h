#pragma once

#include "global.h"
#include "common/time.h"

struct magDev_s;
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *magdev, int16_t *data);
struct accDev_s;
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);

typedef struct accDev_s {
    sensorAccReadFuncPtr readFn; // read 3 axis data function
    int16_t ADCRaw[3];
    timeUs_t lastReadTime;
    float scale;
} accDev_t;

typedef struct gyroDev_s {
    sensorGyroReadFuncPtr readFn; // read 3 axis data function
    int16_t ADCRaw[3];
    timeUs_t lastReadTime;
    float scale;
} gyroDev_t;

typedef struct sensors {
    gyroDev_t gyro;
    accDev_t acc;
} sensors_t;


void InitSonsors(sensors_t * s);
sensors_t * getSonsors(void);
