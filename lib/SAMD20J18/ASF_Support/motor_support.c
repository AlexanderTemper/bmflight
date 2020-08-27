#include <tc.h>
#include <tc_interrupt.h>
#include "motor_support.h"
#include "fc/fc.h"

static struct tc_module tc_instance1, tc_instance2;
static volatile bool tc_instance1_callback_flag; //indicates if tc has finished operation
static volatile bool tc_instance2_callback_flag; //indicates if tc has finished operation

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
static void tc_instance1_callback(struct tc_module * const module_inst_ptr) {
    tc_instance1_callback_flag = true;
}
static void tc_instance2_callback(struct tc_module * const module_inst_ptr) {
    tc_instance2_callback_flag = true;
}

void motor_initialize(void) {
    struct tc_config config_tc1;
    tc_get_config_defaults(&config_tc1);
    config_tc1.counter_size = TC_COUNTER_SIZE_16BIT;
    config_tc1.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
    config_tc1.clock_source = GCLK_GENERATOR_3;
    config_tc1.oneshot = true;
    config_tc1.pwm_channel[0].enabled = true;
    config_tc1.pwm_channel[0].pin_out = PIN_PA22F_TC4_WO0;
    config_tc1.pwm_channel[0].pin_mux = MUX_PA22F_TC4_WO0;
    config_tc1.pwm_channel[1].enabled = true;
    config_tc1.pwm_channel[1].pin_out = PIN_PA23F_TC4_WO1;
    config_tc1.pwm_channel[1].pin_mux = MUX_PA23F_TC4_WO1;
    tc_init(&tc_instance1, TC4, &config_tc1); //PA22,23
    tc_register_callback(&tc_instance1, tc_instance1_callback, TC_CALLBACK_OVERFLOW);
    tc_enable_callback(&tc_instance1, TC_CALLBACK_OVERFLOW);
    tc_instance1_callback_flag = true;

    struct tc_config config_tc2;
    tc_get_config_defaults(&config_tc2);
    config_tc2.counter_size = TC_COUNTER_SIZE_16BIT;
    config_tc2.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
    config_tc2.clock_source = GCLK_GENERATOR_3;
    config_tc2.oneshot = true;
    config_tc2.pwm_channel[0].enabled = true;
    config_tc2.pwm_channel[0].pin_out = PIN_PB00F_TC7_WO0;
    config_tc2.pwm_channel[0].pin_mux = MUX_PB00F_TC7_WO0;
    config_tc2.pwm_channel[1].enabled = true;
    config_tc2.pwm_channel[1].pin_out = PIN_PB01F_TC7_WO1;
    config_tc2.pwm_channel[1].pin_mux = MUX_PB01F_TC7_WO1;
    tc_init(&tc_instance2, TC7, &config_tc2); //PB00,01
    tc_register_callback(&tc_instance2, tc_instance2_callback, TC_CALLBACK_OVERFLOW);
    tc_enable_callback(&tc_instance2, TC_CALLBACK_OVERFLOW);
    tc_instance2_callback_flag = true;
}

void motor_write(motors_command_t *motors) {
    config_t *fc_command = getFcConfig();
    if (tc_instance1_callback_flag && tc_instance2_callback_flag) {
        uint16_t timer_val = COUNT_MAX_16BIT - ONESHOT_MIN_PULSE;
        if (!fc_command->motorOneShot)
            timer_val = COUNT_MAX_16BIT - PWM_MIN_PULSE;

        tc_set_count_value(&tc_instance1, timer_val);
        tc_set_count_value(&tc_instance2, timer_val);

        tc_instance1_callback_flag = false;
        tc_instance2_callback_flag = false;

        if (!fc_command->motorOneShot) {
            tc_set_compare_value(&tc_instance1, 0, COUNT_MAX_16BIT - PWM_MIN_PULSE + (motors->value[0] << 3));
            tc_set_compare_value(&tc_instance1, 1, COUNT_MAX_16BIT - PWM_MIN_PULSE + (motors->value[1] << 3));
            tc_set_compare_value(&tc_instance2, 0, COUNT_MAX_16BIT - PWM_MIN_PULSE + (motors->value[2] << 3));
            tc_set_compare_value(&tc_instance2, 1, COUNT_MAX_16BIT - PWM_MIN_PULSE + (motors->value[3] << 3));
        } else {
            tc_set_compare_value(&tc_instance1, 0, COUNT_MAX_16BIT - ONESHOT_MIN_PULSE + motors->value[0]);
            tc_set_compare_value(&tc_instance1, 1, COUNT_MAX_16BIT - ONESHOT_MIN_PULSE + motors->value[1]);
            tc_set_compare_value(&tc_instance2, 0, COUNT_MAX_16BIT - ONESHOT_MIN_PULSE + motors->value[2]);
            tc_set_compare_value(&tc_instance2, 1, COUNT_MAX_16BIT - ONESHOT_MIN_PULSE + motors->value[3]);
        }
        tc_enable(&tc_instance1);
        tc_enable(&tc_instance2);
    }
}
