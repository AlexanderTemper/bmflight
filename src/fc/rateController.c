#include "fc/rateController.h"

//#include <stdio.h>

static int16_t P_GAIN = 1;
static int16_t lastError[3] = {
    0,
    0,
    0 };

//static int32_t delta1[3], delta2[3];

static int16_t calculatePID(int16_t rateError, axis_e axis) {
    int16_t PTerm = rateError * P_GAIN;
    int16_t ITerm = 0;
    int16_t DTerm = 0;
//    // -----calculate I component
//    // there should be no division before accumulating the error to integrator, because the precision would be reduced.
//    // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
//    // Time correction (to avoid different I scaling for different builds based on average cycle time)
//    // is normalized to cycle time = 2048.
//  errorGyroI[axis] = errorGyroI[axis] + ((RateError * (int32_t) cycleTime) >> 11) * conf.I8[axis];
//
//    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
//    // I coefficient (I8) moved before integration to make limiting independent from PID settings
//    errorGyroI[axis] = constrain(errorGyroI[axis], -2097152, +2097152);
//    ITerm = errorGyroI[axis] >> 13;
//
//    //-----calculate D-term
//    delta = rateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
//    lastError[axis] = RateError;
//
//    // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
//    // would be scaled by different dt each time. Division by dT fixes that.
//    delta = (delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6;
//    // add moving average here to reduce noise
//    deltaSum = delta1[axis] + delta2[axis] + delta;
//    delta2[axis] = delta1[axis];
//    delta1[axis] = delta;
//    DTerm = (deltaSum * conf.D8[axis]) >> 8;

    return PTerm + ITerm + DTerm;
}
void resetRateController() {

}


/**
 * rate_command are in 10 = 1Grad/s
 * @param fcControl
 * @param armed
 * @param gyro
 * @param currentTime
 */
void updateRateController(control_t* fcControl, bool armed, gyroDev_t *gyro, timeUs_t currentTime) {
    if (!armed) {
        //reset I and D term
    }
    int16_t rateError[XYZ_AXIS_COUNT];
    int16_t gyroScaled[XYZ_AXIS_COUNT];

    gyroScaled[X] = gyro->filtered[X] * 10 * gyro->scale;  // 1 Grad == 10
    gyroScaled[Y] = gyro->filtered[Y] * 10 * gyro->scale;  // 1 Grad == 10
    gyroScaled[Z] = gyro->filtered[Z] * 10 * gyro->scale;  // 1 Grad == 10

    rateError[ROLL] = fcControl->rate_command.axis[ROLL] -  gyroScaled[X];
    rateError[PITCH] = fcControl->rate_command.axis[PITCH] - gyroScaled[Y];
    rateError[YAW] = fcControl->rate_command.axis[YAW]  - gyroScaled[Z];
    fcControl->mixer_command.axis[ROLL] = calculatePID(rateError[ROLL], ROLL);
    fcControl->mixer_command.axis[PITCH] = calculatePID(rateError[PITCH], PITCH);
    fcControl->mixer_command.axis[YAW] = calculatePID(rateError[YAW], YAW);

    //printf("%d: in[%d,%d,%d] out[%d,%d,%d]\n", currentTime, rateCommand->axis[ROLL], rateCommand->axis[PITCH], rateCommand->axis[YAW], mixerCommand->axis[ROLL], mixerCommand->axis[PITCH], mixerCommand->axis[YAW]);
}

