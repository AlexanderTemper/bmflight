#include "msp/msp.h"
#include "common/debug.h"

#include "platform.h"

//#define LED_ROT PIN_PA24
//#define LED_GELB PIN_PB02
//#define LED_GRUEN PIN_PA28

int main(void) {
    /********************* Initialize global variables **********************/

    //uint16_t timer = 0;
    /************************* Initializations ******************************/


    platform_initialize();

    /*Initialize MSP */
    serial_initialize();
    msp_initialize();

//    mspPort_t mspPort;
//    mspInit(&mspPort, &serialInstance);
//    initMspDebugPort(&mspPort);
    interrupt_enable();

    // led inti
//    struct port_config config_prt_pin;
//    port_get_config_defaults(&config_prt_pin);
//    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
//    port_pin_set_config(LED_ROT, &config_prt_pin);
//    port_pin_set_config(LED_GELB, &config_prt_pin);
//    port_pin_set_config(LED_GRUEN, &config_prt_pin);
//
//    port_pin_set_output_level(LED_ROT, true);


    /************************** Infinite Loop *******************************/
    printDebug("\r\n\n------- Debug Build 3 -------\n\n\r");
    while (true) {
        processMSP();
    } /* !while (true) */

}
