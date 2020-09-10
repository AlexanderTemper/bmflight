#include "fc/rateController.h"
#include "common/maths.h"
#include "common/debug.h"

#define DEBUG_RATE_PID

// 125000 -> on max error (+-500) we would need 500ms to set back to zero
#define I_WIND_UP_MAX 125000.0f

static float Kp[XYZ_AXIS_COUNT] = {
    1.0f,
    1.0f,
    1.0f };

static float Ki[XYZ_AXIS_COUNT] = {
    1.0f,
    1.0f,
    1.0f };

static float Kd[XYZ_AXIS_COUNT] = {
    1.0f,
    1.0f,
    1.0f };

static float previousError[XYZ_AXIS_COUNT];
static float errorSum[XYZ_AXIS_COUNT];

static void resetRateController(void) {
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
void updateRateController(control_t* fcControl, bool armed, gyroDev_t *gyro, timeUs_t currentTime) {
    if (!armed) {
        resetRateController();
        fcControl->mixer_command.axis[X] = 0;
        fcControl->mixer_command.axis[Y] = 0;
        fcControl->mixer_command.axis[Z] = 0;
        return;
    }

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

    int16_t x = error[X] * Kp[X] + errorSum[X] * Ki[X] + deltaError[X] * Kd[X];
    int16_t y = error[Y] * Kp[Y] + errorSum[Y] * Ki[Y] + deltaError[Y] * Kd[Y];
    int16_t z = error[Z] * Kp[Z] + errorSum[Z] * Ki[Z] + deltaError[Z] * Kd[Z];

    // set new PID Value
    fcControl->mixer_command.axis[X] = constrain(x, -500, +500);
    fcControl->mixer_command.axis[Y] = constrain(y, -500, +500);
    fcControl->mixer_command.axis[Z] = constrain(z, -500, +500);

    // save error
    previousError[X] = error[X];
    previousError[Y] = error[Y];
    previousError[Z] = error[Z];

    // prevent "yaw jump" during yaw correction so the pid controller does not
    //int16_t absYaw =  ABS(fcControl->rate_command.axis[Z]);
    //fcControl->mixer_command.axis[YAW] = constrain(fcControl->mixer_command.axis[YAW], -100 - absYaw, +100 + absYaw);

#ifdef DEBUG_RATE_PID
    rate_controller_debug_data.error[X] = error[X];
    rate_controller_debug_data.error[Y] = error[Y];
    rate_controller_debug_data.error[Z] = error[Z];

    rate_controller_debug_data.errorSum[X] = errorSum[X];
    rate_controller_debug_data.errorSum[Y] = errorSum[Y];
    rate_controller_debug_data.errorSum[Z] = errorSum[Z];

    rate_controller_debug_data.deltaError[X] = deltaError[X];
    rate_controller_debug_data.deltaError[Y] = deltaError[Y];
    rate_controller_debug_data.deltaError[Z] = deltaError[Z];

    rate_controller_debug_data.mixer_command[X] = fcControl->mixer_command.axis[X];
    rate_controller_debug_data.mixer_command[Y] = fcControl->mixer_command.axis[Y];
    rate_controller_debug_data.mixer_command[Z] = fcControl->mixer_command.axis[Z];
#endif
    //printf("%d: in[%d,%d,%d] out[%d,%d,%d]\n", currentTime, rateCommand->axis[ROLL], rateCommand->axis[PITCH], rateCommand->axis[YAW], mixerCommand->axis[ROLL], mixerCommand->axis[PITCH], mixerCommand->axis[YAW]);
}

