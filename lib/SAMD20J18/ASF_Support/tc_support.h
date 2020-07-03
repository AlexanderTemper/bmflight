#ifndef TC_SUPPORT_H_
#define TC_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <tc.h>
#include <tc_interrupt.h>

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Maximum value of a 32-bit counter */
#define COUNT_MAX_32BIT			UINT32_C(0xFFFFFFFF)
/*! TC6 count value to overflow after 1000 milliseconds */
#define TC6_PERIOD_1000MS		COUNT_MAX_32BIT - UINT32_C(500000)
/*! TC6 count value to overflow after 100 milliseconds */
#define TC6_PERIOD_100MS		COUNT_MAX_32BIT - UINT32_C(50000)
/*! TC6 count value to overflow after 10 milliseconds */
#define TC6_PERIOD_10MS			COUNT_MAX_32BIT - UINT32_C(5000)
/*! the value loaded onto TC6 count register */
#define TC6_COUNT_VALUE			TC6_PERIOD_10MS

/*! TC1 count value to overflow after 1 milliseconds */
#define TC1_COUNT_VALUE		0xFFFF - UINT16_C(2000)

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

extern struct tc_module tc1_instance;
extern struct tc_module tc4_instance;
extern struct tc_module tc6_instance;

extern volatile uint32_t tc1_ticks;
extern volatile bool tc4_callback_flag;
extern volatile bool tc6_callback_flag;




/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/
void tc_initialize(void);

void tc1_configure(void);
void tc1_configure_callbacks(void);
void tc1_callback(struct tc_module *const module_inst_ptr);

void tc4_configure(void);
void tc4_configure_callbacks(void);
void tc4_callback(struct tc_module *const module_inst_ptr);
void tc4_wait_for_msec(uint32_t msec);

void tc6_configure(void);
void tc6_configure_callbacks(void);
void tc6_callback(struct tc_module *const module_inst_ptr);
void tc6_stop_counter(void);
void tc6_start_counter(void);

#endif /* TC_SUPPORT_H_ */

