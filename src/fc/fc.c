#include "fc/fc.h"
#include "imu/imu.h"

config_t fc_config;
status_t fc_status;
control_t fc_control;

void initFC(void) {
    // values for Mixer
    fc_config.MINTHROTTLE = 1020;
    fc_config.MAXTHROTTLE = 2000;
    fc_config.MINCOMMAND = 1000;
    fc_config.MIDRC = 1500;
    fc_config.MINCHECK = 1000;
    fc_config.MAXCHECK = 1900;
    fc_config.YAW_DIRECTION = 1;
    fc_config.motorOneShot = true;


    // init status of fc
    fc_status.ARMED = false;

    fc_control.rx.chan[ROLL] = 1500;
    fc_control.rx.chan[PITCH] = 1500;
    fc_control.rx.chan[YAW] = 1500;
    fc_control.rx.chan[THROTTLE] = 1000;
}



