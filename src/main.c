#include "platform.h"
#include "scheduler/scheduler.h"
#include "fc/tasks.h"
#include "fc/fc.h"
#include "imu/imu.h"

int main(void) {

    platform_initialize();
    serial_initialize();
    msp_initialize();
    sensor_initialize();

    interrupt_enable();

    initImu();
    initFC();

    tasksInit();

    // led inti

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
