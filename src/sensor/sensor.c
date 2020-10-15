#include "sensor/sensor.h"
#include "common/debug.h"
#include "eeprom/eeprom_emulation.h"
#include "fc/fc.h"

#define ENABLE_AVG_GYRO

#define CALIBRATING_ACC_CYCLES 500
static uint16_t calibratingA = 0; // take 500 readings for calibration
#define CALIBRATING_GYRO_CYCLES 1000 //2sec on 500Hz
static sensors_t *sensors;

void InitSonsors(sensors_t * s) {
    sensors = s;
    s->gyro.calibration.cyclesRemaining = CALIBRATING_GYRO_CYCLES;
}

sensors_t * getSonsors(void) {
    return sensors;
}

static void performAcclerationCalibration(void) {
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (calibratingA == CALIBRATING_ACC_CYCLES) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += sensors->acc.ADCRaw[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        sensors->acc.ADCRaw[axis] = 0;
    }

    if (calibratingA == 1) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        sensors->acc.accZero[X] = (a[X]) / CALIBRATING_ACC_CYCLES;
        sensors->acc.accZero[Y] = (a[Y]) / CALIBRATING_ACC_CYCLES;
        sensors->acc.accZero[Z] = (a[Z]) / CALIBRATING_ACC_CYCLES - (1 / sensors->acc.scale);
        config_t *conf = getFcConfig();
        conf->ACC_TRIM[X] = sensors->acc.accZero[X];
        conf->ACC_TRIM[Y] = sensors->acc.accZero[Y];
        conf->ACC_TRIM[Z] = sensors->acc.accZero[Z];
        write_EEPROM(conf);
    }

    calibratingA--;
}

/**
 * start calibration
 * @return
 */
void accStartCalibration(void) {
    calibratingA = CALIBRATING_ACC_CYCLES;
}
/**
 * is the acc calibration done
 * @return
 */
bool accIsCalibrationComplete(void) {
    return calibratingA == 0;
}

/**
 * update acc with new values
 * fist perform an calibration
 * @param currentTimeUs
 */
void updateACC(timeUs_t currentTimeUs) {
    sensors->acc.readFn(&sensors->acc);
    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration();
        sensors->acc.data[X] = 0.0f;
        sensors->acc.data[Y] = 0.0f;
        sensors->acc.data[Z] = 0.0f;
        return;
    }

    sensors->acc.data[X] = (sensors->acc.ADCRaw[X] - sensors->acc.accZero[X]) * sensors->acc.scale;
    sensors->acc.data[Y] = (sensors->acc.ADCRaw[Y] - sensors->acc.accZero[Y]) * sensors->acc.scale;
    sensors->acc.data[Z] = (sensors->acc.ADCRaw[Z] - sensors->acc.accZero[Z]) * sensors->acc.scale;
}

typedef struct gyroAvarage_s {
    float accumulatedMeasurements[XYZ_AXIS_COUNT];
    float gyroPrevious[XYZ_AXIS_COUNT];
    timeUs_t accumulatedMeasurementTimeUs;
    timeUs_t accumulationLastTimeSampledUs;
} gyroAvarage_t;
gyroAvarage_t gyroAvarage;

static void performGyroCalibration(void) {
    uint8_t gyroMovementCalibrationThreshold = 48;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (sensors->gyro.calibration.cyclesRemaining == CALIBRATING_GYRO_CYCLES) {
            sensors->gyro.calibration.sum[axis] = 0.0f;
            devClear(&sensors->gyro.calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            sensors->gyro.gyroZero[axis] = 0;
        }

        // Sum up readings
        sensors->gyro.calibration.sum[axis] += sensors->gyro.raw[axis];
        devPush(&sensors->gyro.calibration.var[axis], sensors->gyro.raw[axis]);

        if (sensors->gyro.calibration.cyclesRemaining == 1) {
            // todo check if why using StandardDeviation
            const float stddev = devStandardDeviation(&sensors->gyro.calibration.var[axis]);
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                sensors->gyro.calibration.cyclesRemaining = CALIBRATING_GYRO_CYCLES;
                return;
            }
            sensors->gyro.gyroZero[axis] = sensors->gyro.calibration.sum[axis] / CALIBRATING_GYRO_CYCLES;
        }
    }
    --sensors->gyro.calibration.cyclesRemaining;
}

/**
 * is the gyro calibration done
 * @return
 */
bool isGyroSensorCalibrationComplete(void) {
    return sensors->gyro.calibration.cyclesRemaining == 0;
}

/**
 * update the gyro data with new raw data
 * first an calibration is done
 * the raw values get trimmed with this calibration data
 *
 * @param currentTimeUs
 */
void updateGyro(timeUs_t currentTimeUs) {

    if (!isGyroSensorCalibrationComplete()) {
        performGyroCalibration();
        return;
    }

    int16_t trimmed[XYZ_AXIS_COUNT];
    trimmed[X] = sensors->gyro.raw[X] - sensors->gyro.gyroZero[X];
    trimmed[Y] = sensors->gyro.raw[Y] - sensors->gyro.gyroZero[Y];
    trimmed[Z] = sensors->gyro.raw[Z] - sensors->gyro.gyroZero[Z];

    // optional filter the gyro data here
    sensors->gyro.data[X] = trimmed[X] * sensors->gyro.scale;
    sensors->gyro.data[Y] = trimmed[Y] * sensors->gyro.scale;
    sensors->gyro.data[Z] = trimmed[Z] * sensors->gyro.scale;

#ifdef ENABLE_AVG_GYRO
    const timeDelta_t sampleDeltaUs = currentTimeUs - gyroAvarage.accumulationLastTimeSampledUs;
    gyroAvarage.accumulationLastTimeSampledUs = currentTimeUs;
    gyroAvarage.accumulatedMeasurementTimeUs += sampleDeltaUs;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // integrate using trapezium rule to avoid bias
        gyroAvarage.accumulatedMeasurements[axis] += 0.5f * (gyroAvarage.gyroPrevious[axis] + sensors->gyro.data[axis]) * sampleDeltaUs;
        gyroAvarage.gyroPrevious[axis] = sensors->gyro.data[axis];
    }
#endif
}

bool gyroGetAccumulationAverage(float *accumulationAverage) {
#ifdef ENABLE_AVG_GYRO
    if (gyroAvarage.accumulatedMeasurementTimeUs > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = gyroAvarage.accumulatedMeasurements[axis] / gyroAvarage.accumulatedMeasurementTimeUs;
            gyroAvarage.accumulatedMeasurements[axis] = 0.0f;
        }
        gyroAvarage.accumulatedMeasurementTimeUs = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
#else
    accumulationAverage[X] = sensors->gyro.raw[X];
    accumulationAverage[Y] = sensors->gyro.raw[X];
    accumulationAverage[Z] = sensors->gyro.raw[X];
    return true;
#endif
}
