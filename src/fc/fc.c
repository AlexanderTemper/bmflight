#include "fc/fc.h"
#include "imu/imu.h"
#include "io/pin.h"
#include "sensor/sensor.h"
#include <string.h>

#include "eeprom/eeprom_emulation.h"

config_t fc_config;
status_t fc_status;
control_t fc_control;

config_t default_fc_config = {
    .MINTHROTTLE = 1020,
    .MAXTHROTTLE = 2000,
    .MINCOMMAND = 1000,
    .MIDRC = 1500,
    .MINCHECK = 1000,
    .MAXCHECK = 1900,
    .YAW_DIRECTION = 1,
    .motorOneShot = true,
    .ACC_TRIM = {
        0,
        0,
        0 },
    .CONFIG_VERSION = EEPROM_CONF_VERSION, };

void initFC(void) {

    config_t loaded_config;
    loaded_config.CONFIG_VERSION = 0;
    read_EEPROM(&loaded_config);

    // write default config to eeprom (no config was found or config layout has changed)
    if (loaded_config.CONFIG_VERSION != EEPROM_CONF_VERSION) {
        write_EEPROM(&default_fc_config);
    }
    // copy loaded config to active config
    memcpy(&loaded_config, &fc_config, sizeof(config_t));
    // init status of fc
    fc_status.ARMED = false;
    setStatusLedLevel(ARM_LED, false);
    setStatusLedLevel(CALIBRATION_LED, false);
    setStatusLedLevel(ERROR_LED, false);

    sensors_t *sen = getSonsors();
    //initialize sensor trim values
    sen->acc.accZero[X] = fc_config.ACC_TRIM[X];
    sen->acc.accZero[Y] = fc_config.ACC_TRIM[Y];
    sen->acc.accZero[Z] = fc_config.ACC_TRIM[Z];
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

