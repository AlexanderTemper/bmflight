#include "fc/fc.h"
#include "imu/imu.h"

config_t fc_config;
status_t fc_status;
controller_t fc_controller;
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

    // init controller
    fc_controller.axis[ROLL] = 0;
    fc_controller.axis[PITCH] = 0;
    fc_controller.axis[YAW] = 0;

    fc_control.command[THROTTLE] = 0;
    fc_control.command[ROLL] = 0;
    fc_control.command[PITCH] = 0;
    fc_control.command[YAW] = 0;
}



