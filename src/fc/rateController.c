#include "fc/rateController.h"


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

    fcControl->mixer_command.axis[ROLL] = 0;
    fcControl->mixer_command.axis[PITCH] = 0;
    fcControl->mixer_command.axis[YAW] = 0;

    //printf("%d: in[%d,%d,%d] out[%d,%d,%d]\n", currentTime, rateCommand->axis[ROLL], rateCommand->axis[PITCH], rateCommand->axis[YAW], mixerCommand->axis[ROLL], mixerCommand->axis[PITCH], mixerCommand->axis[YAW]);
}

