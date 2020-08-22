#pragma once

#include "global.h"

#define COUNT_MAX_16BIT         UINT16_C(0xFFFF)
#define PWM_MIN_PULSE           UINT16_C(24000)
#define ONESHOT_MIN_PULSE       UINT16_C(2400)

typedef struct motors_s {
    int16_t value[4];
    bool oneShot;
} motors_t;


// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;



void motorSetup(void (*writeMotorFnP)(motors_t *motors));
/**
 * updates the motor (mixing and writing to motors)
 */
void updateMotors(void);

/**
 * only for testing (sets the motor_disarmed)
 * @param motors
 */
void testMotor(motors_t *motors);


motors_t* getCurrentMotor(void);
