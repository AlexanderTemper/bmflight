#include "fc/attitudeController.h"

static int16_t P_GAIN = 1;
static int16_t calculatePID(int16_t rateError) {
    int16_t PTerm = rateError * P_GAIN;
    int16_t ITerm = 0;
    int16_t DTerm = 0;
    return PTerm + ITerm + DTerm;
}

/**
 * attitude_command is in grad
 * @param fcControl
 * @param armed
 * @param currentTime
 * @param att
 */
void updateAttitudeController(control_t* fcControl, bool armed, timeUs_t currentTime, attitudeEulerAngles_t *att) {
    if(!armed){
        //reset I and D Term
    }
    int16_t rateError[3];
    rateError[ROLL] = fcControl->attitude_command.axis[ROLL] - att->values.roll;
    rateError[PITCH] = fcControl->attitude_command.axis[PITCH] - att->values.pitch;

    //rate_command are in 10 = 1Grad/s
    fcControl->rate_command.axis[ROLL] = fcControl->attitude_command.axis[ROLL] * 10;//calculatePID(rateError[ROLL]);
    fcControl->rate_command.axis[PITCH] = fcControl->attitude_command.axis[PITCH]  * 10;//calculatePID(rateError[PITCH]);
    fcControl->rate_command.axis[YAW] = fcControl->attitude_command.axis[YAW] * 10;
}
