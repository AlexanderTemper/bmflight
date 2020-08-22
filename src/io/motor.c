#include "io/motor.h"
#include "fc/fc.h"
#include "common/maths.h"

static motorMixer_t currentMixer[4];
static motors_t motors_System;
static int16_t motor_disarmed[4];
static void (*writeMotorFuncPtr)(motors_t *motors);

//QuadX mixing table
static const motorMixer_t mixerQuadX[] = {
    {1.0f, -1.0f, 1.0f, -1.0f },          // REAR_R
    {1.0f, -1.0f,-1.0f,  1.0f },          // FRONT_R
    {1.0f,  1.0f, 1.0f,  1.0f },          // REAR_L
    {1.0f,  1.0f,-1.0f, -1.0f },          // FRONT_L
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
void motorSetup(void (*writeMotorFnP)(motors_t *motors)) {
    motors_System.oneShot = getFcConfig()->motorOneShot;
    writeMotorFuncPtr = writeMotorFnP;
    mixerInit();
}

void updateMotors(void) {
    int16_t maxMotor;
    uint32_t i;
    config_t *config = getFcConfig();
    status_t *status = getFcStatus();
    controller_t *controller = getFcController();
    control_t *control = getFcControl();

    // prevent "yaw jump" during yaw correction
    controller->axis[YAW] = constrain(controller->axis[YAW], -100 - ABS(control->command[YAW]), +100 + ABS(control->command[YAW]));

    for (i = 0; i < 4; i++) {
        motors_System.value[i] = control->command[THROTTLE] * currentMixer[i].throttle + controller->axis[PITCH] * currentMixer[i].pitch + controller->axis[ROLL] * currentMixer[i].roll + -config->YAW_DIRECTION * controller->axis[YAW] * currentMixer[i].yaw;
    }

    // get max motor value
    maxMotor = motors_System.value[0];
    for (i = 1; i < 4; i++) {
        if (motors_System.value[i] > maxMotor) {
            maxMotor = motors_System.value[i];
        }
    }

    for (i = 0; i < 4; i++) {
        if (maxMotor > config->MAXTHROTTLE) {    // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors_System.value[i] -= maxMotor - config->MAXTHROTTLE;
        }

        motors_System.value[i] = constrain(motors_System.value[i], config->MINTHROTTLE, config->MAXTHROTTLE);

        if ((control->rx.chan[THROTTLE]) < config->MINCHECK) {
            motors_System.value[i] = config->MINCOMMAND;
        }

        if (!status->ARMED) {
            motors_System.value[i] = motor_disarmed[i];
        }
    }

    //write motor data
    writeMotorFuncPtr(&motors_System);
}

void testMotor(motors_t *motors){
    motor_disarmed[0] = motors->value[0];
    motor_disarmed[1] = motors->value[1];
    motor_disarmed[2] = motors->value[2];
    motor_disarmed[3] = motors->value[3];
}

motors_t* getCurrentMotor(void){
    return &motors_System;
}
