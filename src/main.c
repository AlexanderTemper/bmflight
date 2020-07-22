#include "platform.h"

#include "common/debug.h"
#include "common/time.h"

//#define LED_ROT PIN_PA24
//#define LED_GELB PIN_PB02
//#define LED_GRUEN PIN_PA28

int main(void) {
    platform_initialize();

    serial_initialize();

    msp_initialize();

    sensor_initialize();




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


    timeMs_t now = millis();
    timeMs_t next = now + 1000;
    /************************** Infinite Loop *******************************/
    printDebug("\r\n\n------- Debug Build 3 -------\n\n\r");
    while (true) {
        now = millis();


        sensor_read();

        processMSP();



        if(now >= next){
            next = now + 1000;
            printDebug("sec \n");
        }
    }

}
