#pragma once

#include <stdint.h>
#include <stdbool.h>

struct magDev_s;
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *magdev, int16_t *data);
struct accDev_s;
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);

typedef struct accDev_s {
    sensorAccReadFuncPtr readFn; // read 3 axis data function
    int16_t ADCRaw[3];
    float scale;
} accDev_t;

typedef struct gyroDev_s {
    sensorGyroReadFuncPtr readFn; // read 3 axis data function
    int16_t ADCRaw[3];
    float scale;
} gyroDev_t;
