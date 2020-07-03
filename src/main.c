#include "asf.h"

#include "clock_support.h"
#include "spi_support.h"
#include "tc_support.h"
#include "usart_support.h"

int main(void) {
    /********************* Initialize global variables **********************/

    uint16_t timer = 0;
    /************************* Initializations ******************************/

    /*Initialize SAMD20 MCU*/
    system_init();

    /*Initialize clock module of SAMD20 MCU - Internal RC clock*/
    clock_initialize();

    /*SPI master for communicating with sensors*/
    spi_initialize();

    /*Initialize timers */
    tc_initialize();

    /*Initialize UART */
    usart_initialize();

    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
    /************************** Infinite Loop *******************************/
    while (true) {
		
		
	} /* !while (true) */

}
