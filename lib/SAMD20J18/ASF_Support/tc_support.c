#include "tc_support.h"



struct tc_module tc1_instance;
struct tc_module tc4_instance;
struct tc_module tc6_instance;

volatile uint32_t tc1_ticks;
volatile bool tc4_callback_flag;
volatile bool tc6_callback_flag;

void tc_initialize(void)
{
	tc1_configure ();
	tc1_configure_callbacks();

	tc4_configure ();
	tc4_configure_callbacks();
	
	tc6_configure();
	tc6_configure_callbacks();

}


void tc1_configure (void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;

	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);

	/* set the counter size */
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_2;
	/* set the initial counter register value */
	config_tc.counter_16_bit.value = TC1_COUNT_VALUE;

	tc_init(&tc1_instance,TC1,&config_tc);

	tc1_ticks = 0;
	/* enable the TC module */
	tc_enable(&tc1_instance);
}

void tc1_configure_callbacks (void)
{
	tc_register_callback(&tc1_instance, tc1_callback, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc1_instance, TC_CALLBACK_OVERFLOW);
}

void tc1_callback (struct tc_module *const module_inst_ptr)
{
	tc1_ticks++;
	/* Reset the counter register value */
	tc_set_count_value(&tc1_instance, TC1_COUNT_VALUE);
}

void tc4_configure (void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;
	
	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);
	
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_1;
	
	/* Set the initial compare value */
	config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_0] = 500;
	
	/* initialize TC4 with current configurations */
	tc_init(&tc4_instance, TC4, &config_tc);
	
	/* enable the TC module */
	tc_enable(&tc4_instance);
	
	/* Stop the counter */
	tc_stop_counter(&tc4_instance);
}

void tc4_configure_callbacks (void)
{
	tc_register_callback(&tc4_instance, tc4_callback, TC_CALLBACK_CC_CHANNEL0);
	tc4_callback_flag = false;
	tc_enable_callback(&tc4_instance, TC_CALLBACK_CC_CHANNEL0);
}

void tc4_callback (struct tc_module *const module_inst_ptr)
{
	/* Set the corresponding interrupt flag */
	tc4_callback_flag = true;
}

void tc4_wait_for_msec (uint32_t msec)
{
	/* Set the compare value */
	tc_set_compare_value(&tc4_instance, TC_COMPARE_CAPTURE_CHANNEL_0, (msec *500));
	
	/* start counting */
	tc_start_counter(&tc4_instance);
	
	/* delay until required time is elapsed */
	while (!tc4_callback_flag);
	
	/* stop the counter */
	tc_stop_counter(&tc4_instance);
	/* reset the interrupt flag */
	tc4_callback_flag = false;
}

void tc6_configure(void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;
	
	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);
	
	/* set the counter size */
	config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_1;
	/* set the initial counter register value */
	config_tc.counter_32_bit.value = TC6_COUNT_VALUE;
	
	/* initialize TC6 with current configurations */
	tc_init(&tc6_instance,TC6,&config_tc);
	
	/* enable the TC module */
	tc_enable(&tc6_instance);
}

void tc6_configure_callbacks(void)
{
	tc_register_callback(&tc6_instance, tc6_callback, TC_CALLBACK_OVERFLOW);
	tc6_callback_flag = false;
	tc_enable_callback(&tc6_instance, TC_CALLBACK_OVERFLOW);
}

void tc6_callback (struct tc_module *const module_inst_ptr)
{
	/* Reset the counter register value */
	tc_set_count_value(&tc6_instance, TC6_COUNT_VALUE);
	/* Set the corresponding interrupt flag */
	tc6_callback_flag = true;

}

void tc6_stop_counter (void)
{
	tc_stop_counter(&tc6_instance);
}

void tc6_start_counter (void)
{
	tc_set_count_value(&tc6_instance, TC6_COUNT_VALUE);
	tc_start_counter(&tc6_instance);
}
