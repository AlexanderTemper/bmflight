#include <string.h>

#include "common/maths.h"
#include "config/feature.h"
#include "msp/msp_protocol.h"
#include "msp/msp_commands.h"
#include "fc/fc.h"
#include "fc/tasks.h"
#include "sensor/sensor.h"
#include "scheduler/scheduler.h"
#include "imu/imu.h"

/*********** MSP Functions *****************/
/**
 * function called when an reply is received on the msp interface
 */

static mspResult_e mspFcProcessOutCommandWithArg(uint8_t cmdMSP, sbuf_t *src, sbuf_t *dst) {
    switch (cmdMSP) {
    case MSP_BOXNAMES: {
        //const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
        //serializeBoxReply(dst, page, &serializeBoxNameFn);
    }
        break;
    case MSP_BOXIDS: {
        //const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
        //  serializeBoxReply(dst, page, &serializeBoxPermanentIdFn);
    }
        break;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}
/**
 * generate response for command requestet on msp
 * @param cmdMSP
 * @param dst
 * @return
 */
static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
    switch (cmdMSP) {
    case MSP_BATTERY_CONFIG: //TODO
        sbufWriteU8(dst, (330 + 5) / 10);
        sbufWriteU8(dst, (430 + 5) / 10);
        sbufWriteU8(dst, (350 + 5) / 10);
        sbufWriteU16(dst, 2500);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 330);
        sbufWriteU16(dst, 430);
        sbufWriteU16(dst, 350);
        break;
    case MSP_STATUS_EX:
    case MSP_STATUS: {
//https://github.com/betaflight/betaflight-configurator/blob/master/src/js/msp/MSPHelper.js
//        FC.CONFIG.cycleTime = data.readU16();
//        FC.CONFIG.i2cError = data.readU16();
//        FC.CONFIG.activeSensors = data.readU16();
//        FC.CONFIG.mode = data.readU32();
//        FC.CONFIG.profile = data.readU8();
//        FC.CONFIG.cpuload = data.readU16();
//        if (semver.gte(FC.CONFIG.apiVersion, "1.16.0")) {
//            FC.CONFIG.numProfiles = data.readU8();
//            FC.CONFIG.rateProfile = data.readU8();
//
//            if (semver.gte(FC.CONFIG.apiVersion, "1.36.0")) {
//              // Read flight mode flags
//              var byteCount = data.readU8();
//              for (let i = 0; i < byteCount; i++) {
//                data.readU8();
//              }
//
//              // Read arming disable flags
//              FC.CONFIG.armingDisableCount = data.readU8(); // Flag count
//              FC.CONFIG.armingDisableFlags = data.readU32();
//            }
//
//            TABS.pid_tuning.checkUpdateProfile(true);
//        }
//
//        sensor_status(FC.CONFIG.activeSensors);
//        $('span.i2c-error').text(FC.CONFIG.i2cError);
//        $('span.cycle-time').text(FC.CONFIG.cycleTime);
//        $('span.cpu-load').text(i18n.getMessage('statusbar_cpu_load', [FC.CONFIG.cpuload]));
//        break;

//        { BOXARM, "ARM", 0 },
//        { BOXANGLE, "ANGLE", 1 },
//        { BOXHORIZON, "HORIZON", 2 },

        taskInfo_t taskInfo;
        getTaskInfo(TASK_ATTITUDE, &taskInfo);
        sbufWriteU16(dst, taskInfo.averageDeltaTimeUs); // TODO not Real Cycle Time
        sbufWriteU16(dst, 0); // i2c errors
        //gyro,range,gps,mag,baro,acc
        bool gyro = true;
        bool range = false;
        bool gps = false;
        bool mag = false;
        bool baro = true;
        bool acc = true;
        sbufWriteU16(dst, (acc | baro << 0 | mag << 2 | gps << 3 | range << 4 | gyro << 5)); //sensors

        sbufWriteU32(dst, 0); // unconditional part of flags, first 32 bits

        sbufWriteU8(dst, 0);        //getCurrentPidProfileIndex());
        sbufWriteU16(dst, getSystemLoad());
        if (cmdMSP == MSP_STATUS_EX) {
            sbufWriteU8(dst, 0);        //PID_PROFILE_COUNT);
            sbufWriteU8(dst, 0);        //getCurrentControlRateProfileIndex());
        } else {  // MSP_STATUS
            taskInfo_t taskInfoGyro;
            getTaskInfo(TASK_GYRO, &taskInfoGyro);
            sbufWriteU16(dst, taskInfoGyro.averageDeltaTimeUs);
            sbufWriteU16(dst, 0); // gyro cycle time
        }

        // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
        // header is emited even when all bits fit into 32 bits to allow future extension
        sbufWriteU8(dst, 0);

        // Write arming disable flags
        // 1 byte, flag count
        sbufWriteU8(dst, 0);                //ARMING_DISABLE_FLAGS_COUNT);
        // 4 bytes, flags
        //const uint32_t armingDisableFlags = getArmingDisableFlags();
        sbufWriteU32(dst, 0);               //armingDisableFlags);
    }
        break;
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    case MSP_FC_VARIANT:
        sbufWriteString(dst, FC_VARIANT);
        break;
    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO: { //TODO check info
        sbufWriteData(dst, BOARD_IDENTIFIER, 4);
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection
        sbufWriteU8(dst, 0);  // 0 == FC
        sbufWriteU8(dst, 0); // Target capabilities
        sbufWriteU8(dst, 4); // Name length
        sbufWriteData(dst, "TEST", 4);
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        uint8_t emptySignature[32];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
        sbufWriteU8(dst, 255);
        break;
    }

    case MSP_BUILD_INFO: {
        const char * const shortGitRevision = "NO INFO";
        const char * const buildDate = __DATE__;
        const char * const buildTime = __TIME__;
        sbufWriteData(dst, buildDate, 11);
        sbufWriteData(dst, buildTime, 8);
        sbufWriteData(dst, shortGitRevision, 7);
        break;
    }
    case MSP_ANALOG:
        sbufWriteU8(dst, 3);
        sbufWriteU16(dst, 0); // milliamp hours drawn from battery
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, 0);
        break;
    case MSP_DEBUG: {
        int16_t debug[4]; //todo
        for (int i = 0; i < 4; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;
    }
    case MSP_DEBUGMSG:
        break;
    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;
    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, featureConfig.enabledFeatures);
        break;
    case MSP_RAW_IMU: {
        sensors_t *s = getSonsors();
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf(s->acc.ADCRaw[i] * 100 * s->acc.scale));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf(s->gyro.ADCRaw[i] * 100 * s->gyro.scale));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, 100);
        }
        break;
    }
    case MSP_ATTITUDE: {
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;
    }
    case MSP_NAME: {
        const char pilotname[] = "AlexBETA";
        const int nameLen = strlen(pilotname);
        for (int i = 0; i < nameLen; i++) {
            sbufWriteU8(dst, pilotname[i]);
        }
    }
        break;
    case MSP_MIXER_CONFIG:
            sbufWriteU8(dst, 3); //QUADX
            sbufWriteU8(dst, 0);
            break;
    default:
        return false;
    }
    return true;
}

/**
 * msp commands for setting values
 * @param cmdMSP
 * @param src
 * @return
 */
static mspResult_e mspProcessInCommand(uint8_t cmdMSP, sbuf_t *src) {
    //const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SET_RAW_RC:
        //todo
        break;
    default:
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/**
 * function called when request on msp interface is received
 * @param cmd
 * @param reply
 * @return
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply) {
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else if ((ret = mspFcProcessOutCommandWithArg(cmdMSP, src, dst)) != MSP_RESULT_CMD_UNKNOWN) {
        /* ret */;
    } else {
        ret = mspProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}
void mspFcProcessReply(mspPacket_t *cmd) {
    //no Master so nothing to do here
}
