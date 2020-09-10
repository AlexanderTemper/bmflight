#include "io/motor.h"
#include "fc/fc.h"
#include "common/maths.h"

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static motorMixer_t currentMixer[4];
static command_t *command_System;
static mixer_command_t *mixer_System;
static motors_command_t *motors_System;
static int16_t motor_disarmed[4];
static void (*writeMotorFuncPtr)(motors_command_t *motors);

//QuadX mixing table
static const motorMixer_t mixerQuadX[] = {
    { // throttle , roll , pitch , yaw
        1.0f,
        -1.0f,
        1.0f,
        -1.0f },          // REAR_R
    {
        1.0f,
        -1.0f,
        -1.0f,
        1.0f },          // FRONT_R
    {
        1.0f,
        1.0f,
        1.0f,
        1.0f },          // REAR_L
    {
        1.0f,
        1.0f,
        -1.0f,
        -1.0f },          // FRONT_L
};

/**---- local function ----**/
static void mixerResetMotors(void) {
    int i;
    // set disarmed motor values
    for (i = 0; i < 4; i++) {
        motor_disarmed[i] = getFcConfig()->MINCOMMAND;
    }
}

static void mixerInit(void) {
    int i;
    // copy motor-based mixers
    for (i = 0; i < 4; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetMotors();
}

/**---- function ----**/
void motorSetup(void (*writeMotorFnP)(motors_command_t *motors)) {
    writeMotorFuncPtr = writeMotorFnP;
    control_t *control = getFcControl();
    command_System = &control->fc_command;
    mixer_System = &control->mixer_command;
    motors_System = &control->motor_command;
    mixerInit();
}

void updateMotors(void) {
    int16_t maxMotor, minMotor;
    uint32_t i;
    config_t *config = getFcConfig();
    status_t *status = getFcStatus();

    for (i = 0; i < 4; i++) {
        motors_System->value[i] = command_System->throttle * currentMixer[i].throttle + mixer_System->axis[PITCH] * currentMixer[i].pitch + mixer_System->axis[ROLL] * currentMixer[i].roll + -config->YAW_DIRECTION * mixer_System->axis[YAW] * currentMixer[i].yaw;
    }

    // get max motor value
    maxMotor = motors_System->value[0];
    minMotor = motors_System->value[0];
    for (i = 1; i < 4; i++) {
        if (motors_System->value[i] > maxMotor) {
            maxMotor = motors_System->value[i];
        }
        if (motors_System->value[i] < minMotor) {
            minMotor = motors_System->value[i];
        }
    }

    for (i = 0; i < 4; i++) {
        if (maxMotor > config->MAXTHROTTLE) {    // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors_System->value[i] -= maxMotor - config->MAXTHROTTLE;
        }
        if (minMotor < config->MINTHROTTLE) { // this is a way to still have good gyro corrections if at least one motor reaches its min.
            motors_System->value[i] += config->MINTHROTTLE - minMotor;
        }

        motors_System->value[i] = constrain(motors_System->value[i], config->MINTHROTTLE, config->MAXTHROTTLE);

        if (command_System->throttle < config->MINCHECK) {
            motors_System->value[i] = config->MINCOMMAND;
        }

        if (!status->ARMED) {
            motors_System->value[i] = motor_disarmed[i];
        }
    }
    //write motor data
    writeMotorFuncPtr(motors_System);
}

void testMotor(motors_command_t *motors) {
    motor_disarmed[0] = motors->value[0];
    motor_disarmed[1] = motors->value[1];
    motor_disarmed[2] = motors->value[2];
    motor_disarmed[3] = motors->value[3];
}
