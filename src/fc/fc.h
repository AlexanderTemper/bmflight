#pragma once

#include "global.h"

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

typedef struct config_s {
    int16_t MINTHROTTLE;
    int16_t MAXTHROTTLE;
    int16_t MINCOMMAND;
    int16_t MIDRC;
    int16_t MINCHECK;
    int16_t MAXCHECK;
    uint8_t YAW_DIRECTION;
    bool motorOneShot;
} config_t;

typedef struct status_s {
    bool ARMED;
} status_t;

typedef struct rx_command_s {
    int16_t chan[RX_CHANL_COUNT];
} rx_command_t;

typedef struct command_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t throttle;
    bool arm;
} command_t;

/**
 * commands for motors [1000-2000]
 */
typedef struct motors_command_s {
    int16_t value[4];
} motors_command_t;

/**
 * command for Motors Mixer interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
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
