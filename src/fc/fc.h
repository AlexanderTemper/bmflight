#pragma once

#include "global.h"
#include "fc/controller.h"

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
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
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

typedef struct rx_s {
    int16_t chan[RX_CHANL_COUNT];
} rx_t;

typedef struct control_s {
    int16_t command[4]; //command for Motors interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
    rx_t rx; //rx data
} control_t;

extern config_t fc_config;
extern status_t fc_status;
extern controller_t fc_controller;
extern control_t fc_control;

static inline config_t* getFcConfig(void) {
    return &fc_config;
}

static inline status_t* getFcStatus(void) {
    return &fc_status;
}

static inline controller_t* getFcController(void) {
    return &fc_controller;
}

static inline control_t* getFcControl(void) {
    return &fc_control;
}

void initFC(void);
