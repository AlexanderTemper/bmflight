#include "fc/rateController.h"
#include "common/maths.h"
#include "common/debug.h"
#include "fc/fc.h"

// 125000 -> on max error (+-500) we would need 500ms to set back to zero
#define I_WIND_UP_MAX 125000.0f

static float previousError[XYZ_AXIS_COUNT];
static float errorSum[XYZ_AXIS_COUNT];

void resetRateController(void) {
    previousError[X] = 0;
    previousError[Y] = 0;
    previousError[Z] = 0;

    errorSum[X] = 0;
    errorSum[Y] = 0;
    errorSum[Z] = 0;
}
/**
 * rate_command are in 10 = 1Grad/s
 * @param fcControl
 * @param armed
 * @param gyro
 * @param currentTime
 */
void updateRateController(control_t* fcControl, gyroDev_t *gyro, pid_config_t *pidConfig, timeUs_t currentTime) {

    // calculate error
    float error[XYZ_AXIS_COUNT];
    error[X] = (float) fcControl->rate_command.axis[X] - gyro->data[X] * 10;
    error[Y] = (float) fcControl->rate_command.axis[Y] - gyro->data[Y] * 10;
    error[Z] = (float) fcControl->rate_command.axis[Z] - gyro->data[Z] * 10;

    //integrate error
    errorSum[X] += error[X];
    errorSum[Y] += error[Y];
    errorSum[Z] += error[Z];

    // prevent error windup
    errorSum[X] = constrainf(errorSum[X], -I_WIND_UP_MAX, I_WIND_UP_MAX);
    errorSum[Y] = constrainf(errorSum[Y], -I_WIND_UP_MAX, I_WIND_UP_MAX);
    errorSum[Z] = constrainf(errorSum[Z], -I_WIND_UP_MAX, I_WIND_UP_MAX);

    // calculate derivative coefficients
    int16_t deltaError[XYZ_AXIS_COUNT];
    deltaError[X] = error[X] - previousError[X];
    deltaError[Y] = error[Y] - previousError[Y];
    deltaError[Z] = error[Z] - previousError[Z];

    int16_t pX = error[X] * pidConfig->Kp[X];
    int16_t pY = error[Y] * pidConfig->Kp[Y];
    int16_t pZ = error[Z] * pidConfig->Kp[Z];

    int16_t iX = errorSum[X] * pidConfig->Ki[X];
    int16_t iY = errorSum[Y] * pidConfig->Ki[Y];
    int16_t iZ = errorSum[Z] * pidConfig->Ki[Z];

    int16_t dX = deltaError[X] * pidConfig->Kd[X];
    int16_t dY = deltaError[Y] * pidConfig->Kd[Y];
    int16_t dZ = deltaError[Z] * pidConfig->Kd[Z];

    int16_t sumX = pX + iX + dX;
    int16_t sumY = pY + iY + dY;
    int16_t sumZ = pZ + iZ + dZ;

    // set new PID Value
    fcControl->mixer_command.axis[X] = constrain(sumX, -500, +500);
    fcControl->mixer_command.axis[Y] = constrain(sumY, -500, +500);
    fcControl->mixer_command.axis[Z] = constrain(sumZ, -500, +500);

    // save error
    previousError[X] = error[X];
    previousError[Y] = error[Y];
    previousError[Z] = error[Z];

    // prevent "yaw jump" during yaw correction
    //meint das der Copter nach oben springt wenn man sich schnell drehen will
    //int16_t absYaw =  ABS(fcControl->rate_command.axis[Z]);
    //fcControl->mixer_command.axis[YAW] = constrain(fcControl->mixer_command.axis[YAW], -100 - absYaw, +100 + absYaw);

#ifdef USE_BLACKBOX
    pid_debug_t* pidDebug = &getFcDebug()->pid_debug;
    pidDebug->p[X] = pX;
    pidDebug->p[Y] = pY;
    pidDebug->p[Z] = pZ;

    pidDebug->i[X] = iX;
    pidDebug->i[Y] = iY;
    pidDebug->i[Z] = iZ;

    pidDebug->d[X] = dX;
    pidDebug->d[Y] = dY;
    pidDebug->d[Z] = dZ;
#endif
    //printf("%d: in[%d,%d,%d] out[%d,%d,%d]\n", currentTime, rateCommand->axis[ROLL], rateCommand->axis[PITCH], rateCommand->axis[YAW], mixerCommand->axis[ROLL], mixerCommand->axis[PITCH], mixerCommand->axis[YAW]);
}

