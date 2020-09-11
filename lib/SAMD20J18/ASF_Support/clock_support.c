#include "clock_support.h"

static void clock_configure_dfll(void) {
    /* Structure for DFLL's configuration */
    struct system_clock_source_dfll_config config_dfll;

    /*Get and set the default dfll configuration*/
    system_clock_source_dfll_get_config_defaults(&config_dfll);
    system_clock_source_dfll_set_config(&config_dfll);

    /*Enable DFLL as clock source*/
    enum status_code dfll_status = system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);

    while (dfll_status != STATUS_OK) {
        /* TODO: Handle clock setting error */
    }
}

static void clock_configure_osc8m(void) {
    struct system_clock_source_osc8m_config config_osc8m;
    system_clock_source_osc8m_get_config_defaults(&config_osc8m);
    config_osc8m.prescaler = SYSTEM_OSC8M_DIV_1;
    system_clock_source_osc8m_set_config(&config_osc8m);
}

/**
 * use the full 48Mhz speed of dfll
 */
static void clock_configure_system_clock(void) {
    /* Set waitstates to 2 as we are using a high system clock rate */
    system_flash_set_waitstates(2);
    struct system_gclk_gen_config config_gclock_gen;
    system_gclk_gen_get_config_defaults(&config_gclock_gen);
    config_gclock_gen.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
    system_gclk_gen_set_config(GCLK_GENERATOR_0, &config_gclock_gen);
}


/**
 * 1Mhz clock for counting usec used bye system
 */
static void clock_configure_gclk_generator_1(void) {
    struct system_gclk_gen_config gclock_gen_conf;
    system_gclk_gen_get_config_defaults(&gclock_gen_conf);
    gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC8M;
    gclock_gen_conf.division_factor = 8;
    system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclock_gen_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_1);
}

/**
 * 8Mhz clock for serial interface
 */
static void clock_configure_gclk_generator_2(void) {
    struct system_gclk_gen_config gclock_gen_conf;
    system_gclk_gen_get_config_defaults(&gclock_gen_conf);
    gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC8M;
    gclock_gen_conf.division_factor = 1;
    system_gclk_gen_set_config(GCLK_GENERATOR_2, &gclock_gen_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_2);
}



/**
 * 8MHz clock for PWM to motor
 */
static void clock_configure_gclk_generator_3(void) {
    struct system_gclk_gen_config gclock_gen_conf;
    system_gclk_gen_get_config_defaults(&gclock_gen_conf);
    gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC8M;
    system_gclk_gen_set_config(GCLK_GENERATOR_3, &gclock_gen_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_3);
}

void clock_initialize(void) {
    clock_configure_dfll();
    clock_configure_osc8m();
    clock_configure_system_clock();
    clock_configure_gclk_generator_1();
    clock_configure_gclk_generator_2();
    clock_configure_gclk_generator_3();
}
