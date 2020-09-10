#pragma once

#include "global.h"
#include "common/time.h"
#include "common/maths.h"

struct magDev_s;
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *magdev, int16_t *data);
struct accDev_s;
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);

typedef struct accDev_s {
    sensorAccReadFuncPtr readFn; // read 3 axis data function

    int16_t ADCRaw[XYZ_AXIS_COUNT];
    float data[XYZ_AXIS_COUNT];

    float scale;
    int16_t accZero[XYZ_AXIS_COUNT];
    timeUs_t lastReadTime;

} accDev_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroDev_s {
    sensorGyroReadFuncPtr readFn; // read 3 axis data function
    int16_t raw[XYZ_AXIS_COUNT];
    timeUs_t lastReadTime;
    float scale;
    float data[XYZ_AXIS_COUNT];
    int16_t gyroZero[XYZ_AXIS_COUNT];
    gyroCalibration_t calibration;
} gyroDev_t;

typedef struct sensors {
    gyroDev_t gyro;
    accDev_t acc;
} sensors_t;

void InitSonsors(sensors_t * s);
sensors_t * getSonsors(void);
bool isGyroSensorCalibrationComplete(void);
void updateGyro(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulationAverage);
void accStartCalibration(void);
bool accIsCalibrationComplete(void);
void updateACC(timeUs_t currentTimeUs);
