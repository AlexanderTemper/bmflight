#include "blackbox/blackbox.h"
#include "fc/fc.h"
#include "imu/imu.h"
#include "io/pin.h"
#include "sensor/sensor.h"
#include "eeprom/eeprom_emulation.h"
#include "fc/tasks.h"

#include <string.h>

config_t fc_config;
status_t fc_status;
control_t fc_control;

static config_t default_fc_config = {
    .MINTHROTTLE = 1020,
    .MAXTHROTTLE = 2000,
    .MINCOMMAND = 1000,
    .MIDRC = 1500,
    .MINCHECK = 1000,
    .MAXCHECK = 1900,
    .YAW_DIRECTION = 1,
    .motorOneShot = true,
    .ARM_TIMEOUT_US = 500000,
    .MAX_ARMING_ANGLE = 25,
    .rate_controller_config = {
        .Kp = {
            0.1f,
            0.2f,
            0.3f },
        .Ki = {
            0.01f,
            0.02f,
            0.03f },
        .Kd = {
            0.04f,
            0.05f,
            0.06f } },
    .levelGain = 0.0f,
    .deciLevelAngleLimit = 150,
    .ACC_TRIM = {
        -38,
        -19,
        42 },
    .CONFIG_VERSION = EEPROM_CONF_VERSION,
    .blackboxEnabled = false,
    .PILOTNAME = {
        'T',
        'e',
        'm',
        'p',
        'e',
        'r',
        'A',
        't',
        'u',
        'r',
        0,
        0,
        0,
        0,
        0,
        0 }, };

void initFC(void) {

    config_t init_config;
    init_config.CONFIG_VERSION = 0;

    config_t *loaded_config = &init_config;
    read_EEPROM(loaded_config);

    // write default config to eeprom (no config was found or config layout has changed)
    if (loaded_config->CONFIG_VERSION != EEPROM_CONF_VERSION) {
        write_EEPROM(&default_fc_config); //write default
        loaded_config = &default_fc_config;
    }
    // copy loaded config to active config
    memcpy(&fc_config, loaded_config, sizeof(config_t));
    // init status of fc
    fc_status.ARMED = false;
    fc_status.ANGLE = false;

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
#ifdef USE_BLACKBOX
    //init blackBox
    blackboxInit(fc_config.blackboxEnabled);
#endif
}

void rebootFC(void) {
    initFC();
    tasksInit();
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

