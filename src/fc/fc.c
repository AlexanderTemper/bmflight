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

    //initialize rx channels
    resetRx();
}

/**
 * reset the rx input buffers
 * (startup or on timeout)
 */
void resetRx(void) {
    fc_control.rx.chan[ROLL] = fc_config.MIDRC;
    fc_control.rx.chan[PITCH] = fc_config.MIDRC;
    fc_control.rx.chan[YAW] = fc_config.MIDRC;
    fc_control.rx.chan[THROTTLE] = fc_config.MINCOMMAND;
    fc_control.rx.chan[AUX1] = fc_config.MINCOMMAND;
    fc_control.rx.chan[AUX2] = fc_config.MINCOMMAND;
}

