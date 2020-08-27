#pragma once

#include "global.h"
#include "fc/fc.h"

#define COUNT_MAX_16BIT         UINT16_C(0xFFFF)
#define PWM_MIN_PULSE           UINT16_C(24000)
#define ONESHOT_MIN_PULSE       UINT16_C(2400)


void motorSetup(void (*writeMotorFnP)(motors_command_t *motors));
/**
 * updates the motor (mixing and writing to motors)
 */
void updateMotors(void);

/**
 * only for testing (sets the motor_disarmed)
 * @param motors
 */
void testMotor(motors_command_t *motors);
