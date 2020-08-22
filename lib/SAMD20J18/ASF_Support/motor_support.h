#pragma once

#include "global.h"
#include "io/motor.h"
/**
 * init the timer counter
 */
void motor_initialize(void);

/**
 * write the motor values to the timer counter register
 */
void motor_write(motors_t *motors);
