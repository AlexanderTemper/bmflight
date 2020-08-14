#include "platform.h"

#include "common/time.h"
#include "scheduler/scheduler.h"
#include "fc/tasks.h"
#include "fc/fc.h"


//#include <stdio.h>
//#include "dyad.h"
//#define LED_ROT PIN_PA24
//#define LED_GELB PIN_PB02
//#define LED_GRUEN PIN_PA28

int main(void) {

    platform_initialize();
    serial_initialize();
    msp_initialize();
    sensor_initialize();

    interrupt_enable();

    initFC(); //set IMU

    tasksInit();

    // led inti
//    struct port_config config_prt_pin;
//    port_get_config_defaults(&config_prt_pin);
//    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
//    port_pin_set_config(LED_ROT, &config_prt_pin);
//    port_pin_set_config(LED_GELB, &config_prt_pin);
//    port_pin_set_config(LED_GRUEN, &config_prt_pin);
//
//    port_pin_set_output_level(LED_ROT, true);
//
//    //
//            processMSP();
//            printDebug("hallo");
    /************************** Infinite Loop *******************************/
    while (true) {
        scheduler();
        delayNanoSeconds(50);
    }

}
