#pragma once

#include "global.h"
#include "common/time.h"

typedef enum BlackboxDevice {
    BLACKBOX_DEVICE_NONE = 0,
    BLACKBOX_DEVICE_SERIAL = 3
} BlackboxDevice_e;

typedef enum BlackboxMode {
    BLACKBOX_MODE_NORMAL = 0,
} BlackboxMode;



typedef struct blackboxConfig_s {
    uint16_t p_ratio; // I-frame interval / P-frame interval
    uint8_t device;
    uint8_t record_acc;
    uint8_t mode;
} blackboxConfig_t;

extern blackboxConfig_t blackboxConfig_System;
blackboxConfig_t* blackboxConfig(void);


//union flightLogEventData_u;
//void blackboxLogEvent(FlightLogEvent event, union flightLogEventData_u *data);

void blackboxInit(bool enabled);
void blackboxUpdate(timeUs_t currentTimeUs);
void blackboxStart(void);
void blackboxStop(void);
uint8_t blackboxGetRateDenom(void);

