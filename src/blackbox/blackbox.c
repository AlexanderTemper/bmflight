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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "global.h"

#ifdef USE_BLACKBOX

#include "blackbox.h"

#define DEFAULT_BLACKBOX_DEVICE  BLACKBOX_DEVICE_NONE
// number of flight loop iterations before logging I-frame
static int16_t blackboxIInterval = 0;
//number of flight loop iterations before logging P-frame
static int16_t blackboxPInterval = 0;
static int32_t blackboxSInterval = 0;

blackboxConfig_t blackboxConfig_System = {
    .p_ratio = 16,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .record_acc = 1,
    .mode = BLACKBOX_MODE_NORMAL };

blackboxConfig_t* blackboxConfig(void) {
    return &blackboxConfig_System;
}

uint8_t blackboxGetRateDenom(void) {
    return blackboxPInterval;
}
/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(bool enabled) {
    if (enabled) {
        blackboxConfig_System.device = BLACKBOX_DEVICE_SERIAL;
    }
    //blackboxResetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    blackboxIInterval = (uint16_t) ((32 * TARGET_LOOP_HZ) / 1000);

    if (blackboxConfig()->p_ratio == 0) {
        blackboxPInterval = 0; // blackboxPInterval not used when p_ratio is zero, so just set it to zero
    } else if (blackboxConfig()->p_ratio > blackboxIInterval && blackboxIInterval >= 32) {
        blackboxPInterval = 1;
    } else {
        blackboxPInterval = blackboxIInterval / blackboxConfig()->p_ratio;
    }
//    if (blackboxConfig()->device) {
//        blackboxSetState(BLACKBOX_STATE_STOPPED);
//    } else {
//        blackboxSetState(BLACKBOX_STATE_DISABLED);
//    }
    blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
}
#endif
