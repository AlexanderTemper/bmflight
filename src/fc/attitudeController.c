#include "fc/attitudeController.h"

static int16_t P_GAIN = 1;
static int16_t calculatePID(int16_t rateError) {
    int16_t PTerm = rateError * P_GAIN;
    int16_t ITerm = 0;
    int16_t DTerm = 0;
    return PTerm + ITerm + DTerm;
}

void updateAttitudeController(attitude_command_t *attitudeCommand, attitudeEulerAngles_t *att, rate_command_t *rateCommand, timeUs_t currentTime) {
    int16_t rateError[3];
    rateError[ROLL] = attitudeCommand->axis[ROLL] - att->values.roll;
    rateError[PITCH] = attitudeCommand->axis[PITCH] - att->values.pitch;

    rateCommand->axis[ROLL] = calculatePID(rateError[ROLL]);
    rateCommand->axis[PITCH] = calculatePID(rateError[PITCH]);
    rateCommand->axis[YAW] = attitudeCommand->axis[YAW];
}
