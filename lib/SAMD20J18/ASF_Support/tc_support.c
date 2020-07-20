#include "tc_support.h"
#include <tc.h>
#include <tc_interrupt.h>

static struct tc_module muTimer;
static uint32_t muTimerOverflow;
//volatile bool tc6_callback_flag;

static void muTimer_configure(void) {
    /* TC's configuration structure */
    struct tc_config config_tc;

    /* Get TC configuration default */
    tc_get_config_defaults(&config_tc);

    /* set the counter size */
    config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
    /* set TC GLCK */
    config_tc.clock_source = GCLK_GENERATOR_1;
    /* set the initial counter register value */
    config_tc.counter_32_bit.value = 0;
    muTimerOverflow = 0;
    tc_init(&muTimer, TC0, &config_tc);

    /* enable the TC module */
    tc_enable(&muTimer);
}
// 1.19304647 hours
static void muTimer_callback(struct tc_module * const module_inst_ptr) {
    /* Reset the counter register value */
    muTimerOverflow++;
    tc_set_count_value(&muTimer, 0);
}

static void muTimer_configure_callbacks(void) {
    tc_register_callback(&muTimer, muTimer_callback, TC_CALLBACK_OVERFLOW);
    tc_enable_callback(&muTimer, TC_CALLBACK_OVERFLOW);
}

void tc_initialize(void) {
    muTimer_configure();
    muTimer_configure_callbacks();
}

void wait_for_msec(uint32_t msec) {

    uint32_t waitUntil = millis_samd20j18() + msec;

    while (millis_samd20j18() <= waitUntil) {
    }
}

/**
 * returns the time in ms since start
 * @return
 */
uint32_t millis_samd20j18(void) {
   uint32_t slow = (4294967.296 * muTimerOverflow) + (muTimer.hw->COUNT32.COUNT.reg / 1000); //TODO make faster

    return slow;
}

/**
 * returns the time in us since start
 * @return
 */
uint32_t micros_samd20j18(void) {
    return muTimer.hw->COUNT32.COUNT.reg;
}
