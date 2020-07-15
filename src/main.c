#include "asf.h"

#include "clock_support.h"
#include "spi_support.h"
#include "tc_support.h"
#include "usart_support.h"

#include "drivers/serial.h"
#include "msp/msp.h"

#define LED_ROT PIN_PA24
#define LED_GELB PIN_PB02
#define LED_GRUEN PIN_PA28

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

    /*Initialize MSP */
    serialPort_t serialInstance;
    initSerial(&serialInstance, &mspSerialUartWriteCallback);
    usart_initialize(&serialInstance);

    mspPort_t mspPort;
    mspInit(&mspPort, &serialInstance);


    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */

    // led inti
    struct port_config config_prt_pin;
    port_get_config_defaults(&config_prt_pin);
    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(LED_ROT, &config_prt_pin);
    port_pin_set_config(LED_GELB, &config_prt_pin);
    port_pin_set_config(LED_GRUEN, &config_prt_pin);

    port_pin_set_output_level(LED_ROT, true);
    mspSerialPush(&mspPort, 1, 0, 0, MSP_DIRECTION_REQUEST);
    /************************** Infinite Loop *******************************/
    while (true) {

    } /* !while (true) */

}
