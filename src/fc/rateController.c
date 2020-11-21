#include "fc/rateController.h"
#include "common/maths.h"
#include "common/debug.h"
#include "fc/fc.h"

static int16_t previousError[XYZ_AXIS_COUNT];
static int32_t errorGyroI[XYZ_AXIS_COUNT];
static int32_t delta1[3], delta2[3];
timeUs_t lastTime;

void resetRateController(void) {
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        previousError[i] = 0;
        errorGyroI[i] = 0;
        delta1[i] = 0;
        delta2[i] = 0;
    }
}

static int16_t updateOnAxis(int16_t setPoint, int16_t current, uint16_t kp, uint16_t ki, uint16_t kd, axis_e axis, timeDelta_t dt) {

    current = constrain(current, -8000, +8000); //limit the gyro to +- 800 degrees
    // calculate error
    int16_t error = setPoint - current;
    // -----calculate I component
    // there should be no division before accumulating the error to integrator, because the precision would be reduced.
    // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
    // Time correction (to avoid different I scaling for different builds based on average cycle time)
    // is normalized to cycle time = 2048.
    errorGyroI[axis] = errorGyroI[axis] + ((error * (int32_t) dt) >> 11) * ki;

    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    errorGyroI[axis] = constrain(errorGyroI[axis], -2097152, +2097152);

    //-----calculate D-term
    // dT is fixed to approximative 2048us (488Hz)
    int32_t delta = (error - previousError[axis]);  //max 8000
    previousError[axis] = error;

    // add moving average here to reduce noise
    int32_t deltaSum = delta1[axis] + delta2[axis] + delta;  //max 8000*3 = 240000
    delta2[axis] = delta1[axis];
    delta1[axis] = delta;

    // calculate terms
    int16_t p = (error * kp) >> 8; // /256
    int16_t i = errorGyroI[axis] >> 13;   // /8292
    int16_t d = (deltaSum * kd) >> 8; // /256

    //printf("%d:: delta1[%d] delta2[%d] delta[%d] deltaSum[%d]\n",axis,delta1[axis],delta2[axis],delta,deltaSum);

    //printf("%d:: error[%d] setPoint[%d] current[%d] p[%d]\n",axis,error,setPoint,current,p);

#ifdef USE_BLACKBOX
    pid_debug_t* pidDebug = &getFcDebug()->pid_debug;
    pidDebug->p[axis] = p;
    pidDebug->i[axis] = i;
    pidDebug->d[axis] = d;
#endif

    return p + i + d;
}

/**
 * rate_command are in 10 = 1Grad/s
 * @param fcControl
 * @param armed
 * @param gyro
 * @param currentTime
 */
void updateRateController(control_t* fcControl, gyroDev_t *gyro, pid_config_t *pidConfig, timeUs_t currentTime) {

    timeDelta_t dt = cmpTimeUs(currentTime, lastTime);

    int16_t sumX = updateOnAxis(fcControl->rate_command.axis[X], lrintf(gyro->data[X] * 10), pidConfig->Kp[X], pidConfig->Ki[X], pidConfig->Kd[X], X, dt);
    int16_t sumY = updateOnAxis(fcControl->rate_command.axis[Y], lrintf(gyro->data[Y] * 10), pidConfig->Kp[Y], pidConfig->Ki[Y], pidConfig->Kd[Y], Y, dt);
    int16_t sumZ = updateOnAxis(fcControl->rate_command.axis[Z], lrintf(gyro->data[Z] * 10), pidConfig->Kp[Z], pidConfig->Ki[Z], pidConfig->Kd[Z], Z, dt);

    lastTime = currentTime;
    // set new PID Value
    fcControl->mixer_command.axis[X] = constrain(sumX, -500, +500);
    fcControl->mixer_command.axis[Y] = constrain(sumY, -500, +500);
    fcControl->mixer_command.axis[Z] = constrain(sumZ, -500, +500);

    // prevent "yaw jump" during yaw correction
    //meint das der Copter nach oben springt wenn man sich schnell drehen will
    //int16_t absYaw =  ABS(fcControl->rate_command.axis[Z]);
    //fcControl->mixer_command.axis[YAW] = constrain(fcControl->mixer_command.axis[YAW], -100 - absYaw, +100 + absYaw);
    //printf("%d: in[%d,%d,%d] out[%d,%d,%d]\n", currentTime, rateCommand->axis[ROLL], rateCommand->axis[PITCH], rateCommand->axis[YAW], mixerCommand->axis[ROLL], mixerCommand->axis[PITCH], mixerCommand->axis[YAW]);

}

