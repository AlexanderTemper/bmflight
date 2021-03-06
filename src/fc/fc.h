#pragma once

#include "global.h"
#include "common/time.h"

#define FC_VERSION_MAJOR 0
#define FC_VERSION_MINOR 0
#define FC_VERSION_PATCH_LEVEL 1
#define FC_VARIANT "BTFL"
#define BOARD_IDENTIFIER "BMFC"

typedef enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    RX_CHANL_COUNT
} rc_alias_e;

#define EEPROM_CONF_VERSION 6 //make sure to change if struct changes

typedef struct pid_config_s {
    uint8_t Kp[XYZ_AXIS_COUNT];
    uint8_t Ki[XYZ_AXIS_COUNT];
    uint8_t Kd[XYZ_AXIS_COUNT];
} pid_config_t;

typedef struct config_s {
    uint16_t CONFIG_VERSION;
    char PILOTNAME[16];
    int16_t MINTHROTTLE;
    int16_t MAXTHROTTLE;
    int16_t MINCOMMAND;
    int16_t MIDRC;
    int16_t MINCHECK;
    int16_t MAXCHECK;
    int8_t YAW_DIRECTION;
    int16_t ACC_TRIM[XYZ_AXIS_COUNT];
    uint8_t RC_RATES[XYZ_AXIS_COUNT];
    int32_t ARM_TIMEOUT_US;
    uint8_t MAX_ARMING_ANGLE;
    pid_config_t rate_controller_config;
    float levelGain;
    uint16_t deciLevelAngleLimit;
    bool motorOneShot;
    bool blackboxEnabled;
} config_t;

typedef struct status_s {
    bool ARMED;
    bool ANGLE;
} status_t;

typedef struct rx_command_s {
    int16_t chan[RX_CHANL_COUNT];
    timeUs_t lastReceived;
} rx_command_t;

typedef struct command_s {
    int16_t roll;     //[-500;+500]
    int16_t pitch;    //[-500;+500]
    int16_t yaw;      //[-500;+500]
    int16_t throttle; // [1000;2000]
    bool arm;
} command_t;

/**
 * commands for motors [1000-2000]
 */
typedef struct motors_command_s {
    int16_t value[4];
} motors_command_t;

/**
 * command [-500;+500] for the axis
 */
typedef struct mixer_command_s {
    int16_t axis[3];
} mixer_command_t;

/**
 * command for rate controller
 */
typedef struct rate_command_s {
    int16_t axis[3];
} rate_command_t;

/**
 * command for attitude controller
 */
typedef struct attitude_command_s {
    int16_t axis[3];
} attitude_command_t;

typedef struct control_s {
    rx_command_t rx;                     //raw data form rx
    command_t fc_command;                //processed data from rx

    attitude_command_t attitude_command; //data provided to attitude controller
    rate_command_t rate_command;         //data provided to rate controller
    mixer_command_t mixer_command;       //data provided to mixer
    motors_command_t motor_command;      //data provided to motors
} control_t;

extern config_t fc_config;
extern status_t fc_status;
extern control_t fc_control;

typedef struct pid_debug_s {
    int16_t p[XYZ_AXIS_COUNT];
    int16_t i[XYZ_AXIS_COUNT];
    int16_t d[XYZ_AXIS_COUNT];
} pid_debug_t;

typedef struct fc_debug_s {
    pid_debug_t pid_debug;
} fc_debug_t;

extern fc_debug_t fc_debug;

static inline fc_debug_t* getFcDebug(void) {
    return &fc_debug;
}

static inline config_t* getFcConfig(void) {
    return &fc_config;
}

static inline status_t* getFcStatus(void) {
    return &fc_status;
}

static inline control_t* getFcControl(void) {
    return &fc_control;
}

void initFC(void);
void rebootFC(void);
void resetRx(void);
