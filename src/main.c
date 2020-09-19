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

    /************************** Infinite Loop *******************************/
    while (true) {
        scheduler();
        delayNanoSeconds(50);
    }

}
