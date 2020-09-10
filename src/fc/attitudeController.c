#include "fc/attitudeController.h"

/**
 * attitude_command is in grad
 * @param fcControl
 * @param armed
 * @param currentTime
 * @param att
 */
void updateAttitudeController(control_t* fcControl, config_t *fcConfig, timeUs_t currentTime, attitudeEulerAngles_t *att) {
    // calculate desired angle (limited by max inclination)  decigrad (1Grad = 10)
    float angleRoll = fcConfig->deciLevelAngleLimit * (float) fcControl->attitude_command.axis[ROLL] / 500.0f;
    float anglePitch = fcConfig->deciLevelAngleLimit * (float) fcControl->attitude_command.axis[PITCH] / 500.0f;

    //calculate error angle
    int16_t rollSetpoint = fcConfig->levelGain * (angleRoll - att->values.roll);
    int16_t pitchSetpoint = fcConfig->levelGain * (anglePitch - att->values.pitch);

    fcControl->rate_command.axis[X] = constrain(rollSetpoint, -fcConfig->deciLevelAngleLimit, fcConfig->deciLevelAngleLimit);
    fcControl->rate_command.axis[Y] = constrain(pitchSetpoint, -fcConfig->deciLevelAngleLimit, fcConfig->deciLevelAngleLimit);
    fcControl->rate_command.axis[Z] = fcControl->attitude_command.axis[Z];
}
