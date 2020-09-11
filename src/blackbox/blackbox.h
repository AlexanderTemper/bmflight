/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "global.h"
#include "common/time.h"

typedef enum BlackboxDevice {
    BLACKBOX_DEVICE_NONE = 0,
    BLACKBOX_DEVICE_SERIAL = 3
} BlackboxDevice_e;

typedef enum BlackboxMode {
    BLACKBOX_MODE_NORMAL = 0,
    BLACKBOX_MODE_MOTOR_TEST,
    BLACKBOX_MODE_ALWAYS_ON
} BlackboxMode;

typedef enum FlightLogEvent {
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
    FLIGHT_LOG_EVENT_LOG_END = 255
} FlightLogEvent;

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

