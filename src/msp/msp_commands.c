#include <string.h>

#include "blackbox/blackbox.h"
#include "common/maths.h"
#include "common/debug.h"
#include "msp/msp_protocol.h"
#include "msp/msp_commands.h"
#include "fc/fc.h"
#include "fc/tasks.h"
#include "sensor/sensor.h"
#include "scheduler/scheduler.h"
#include "imu/imu.h"
#include "io/motor.h"
#include "eeprom/eeprom_emulation.h"

/*********** MSP Functions *****************/
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
        getTaskInfo(TASK_LOOP, &taskInfo);
        sbufWriteU16(dst, taskInfo.latestDeltaTimeUs);
        sbufWriteU16(dst, 0); // i2c errors
        //gyro,range,gps,mag,baro,acc
        bool gyro = true;
        bool range = false;
        bool gps = false;
        bool mag = false;
        bool baro = false;
        bool acc = true;
        sbufWriteU16(dst, (acc | baro << 1 | mag << 2 | gps << 3 | range << 4 | gyro << 5)); //sensors

        sbufWriteU32(dst, 0); // unconditional part of flags, first 32 bits

        sbufWriteU8(dst, 0);        //getCurrentPidProfileIndex());
        sbufWriteU16(dst, getSystemLoad());
        if (cmdMSP == MSP_STATUS_EX) {
            sbufWriteU8(dst, 0);        //PID_PROFILE_COUNT);
            sbufWriteU8(dst, 0);        //getCurrentControlRateProfileIndex());
        } else {  // MSP_STATUS
            taskInfo_t taskInfoGyro;
            getTaskInfo(TASK_LOOP, &taskInfoGyro);
            sbufWriteU16(dst, taskInfoGyro.latestDeltaTimeUs);
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
    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, 16);  // was gyroConfig()->gyro_sync_denom - removed in API 1.43
        sbufWriteU8(dst, 1);  //pidConfig()->pid_process_denom);
        sbufWriteU8(dst, 0);  //motorConfig()->dev.useUnsyncedPwm);
        sbufWriteU8(dst, getFcConfig()->motorOneShot);
        sbufWriteU16(dst, 0); // motorConfig()->dev.motorPwmRate);
        sbufWriteU16(dst, 0); //  motorConfig()->digitalIdleOffsetValue);
        sbufWriteU8(dst, 0); // DEPRECATED: gyro_use_32kHz
        sbufWriteU8(dst, 0); //  motorConfig()->dev.motorPwmInversion);
        sbufWriteU8(dst, 0); // gyroConfig()->gyro_to_use);
        sbufWriteU8(dst, 0); //  gyroConfig()->gyro_high_fsr);
        sbufWriteU8(dst, 0); //  gyroConfig()->gyroMovementCalibrationThreshold);
        sbufWriteU16(dst, 2000); // gyroConfig()->gyroCalibrationDuration);
        sbufWriteU16(dst, 0); //  gyroConfig()->gyro_offset_yaw);
        sbufWriteU8(dst, 0); //  gyroConfig()->checkOverflow);
        //Added in MSP API 1.42
        sbufWriteU8(dst, 0); // systemConfig()->debug_mode);
        sbufWriteU8(dst, 4);

        break;
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    case MSP_BOXNAMES:
        break;
#ifdef USE_BLACKBOX
    case MSP_BLACKBOX_CONFIG:
        sbufWriteU8(dst, 1); //Blackbox supported
        sbufWriteU8(dst, blackboxConfig()->device);
        sbufWriteU8(dst, 1); // Rate numerator, not used anymore
        sbufWriteU8(dst, blackboxGetRateDenom());
        sbufWriteU16(dst, blackboxConfig()->p_ratio);
        break;
#endif
    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, 0);
        break;
    case MSP_PIDNAMES: {
        const char pidNames[] = "ROLL;PITCH;YAW;LEVEL;MAG;";
        for (const char *c = pidNames; *c; c++) {
            sbufWriteU8(dst, *c);
        }
    }
        break;
    case MSP_PID_ADVANCED: {
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // was pidProfile.yaw_p_limit
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); //currentPidProfile->vbatPidCompensation);
        sbufWriteU8(dst, 0); //currentPidProfile->feedForwardTransition);
        sbufWriteU8(dst, 0); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU16(dst, 0); //currentPidProfile->rateAccelLimit);
        sbufWriteU16(dst, 0); //currentPidProfile->yawRateAccelLimit);
        sbufWriteU8(dst, getFcConfig()->deciLevelAngleLimit / 10);
        sbufWriteU8(dst, 0); // was pidProfile.levelSensitivity
        sbufWriteU16(dst, 0); //currentPidProfile->itermThrottleThreshold);
        sbufWriteU16(dst, 0); //currentPidProfile->itermAcceleratorGain);
        sbufWriteU16(dst, 0); // was currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, 0); //currentPidProfile->iterm_rotation);
    }
        break;
    case MSP_PID: {
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, getFcConfig()->rate_controller_config.Kp[i] * 100.0f);
            sbufWriteU8(dst, getFcConfig()->rate_controller_config.Ki[i] * 100.0f);
            sbufWriteU8(dst, getFcConfig()->rate_controller_config.Kd[i] * 100.0f);
        }
        //ATT
        sbufWriteU8(dst, getFcConfig()->levelGain * 10);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);

    }
        break;

    case MSP_RC_DEADBAND: {
        uint8_t deadband = 1;     // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
        uint8_t yaw_deadband = 1; // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
        sbufWriteU8(dst, deadband);
        sbufWriteU8(dst, yaw_deadband);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
    }
        break;
    case MSP_RC_TUNING: {
        uint8_t rcRates[3] = {
            100,
            100,
            100 };
        sbufWriteU8(dst, rcRates[X]);
        sbufWriteU8(dst, 0); //rcExpo
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, 0); // R,P,Y see flight_dynamics_index_t
        }
        sbufWriteU8(dst, 0); //dynThrPID
        sbufWriteU8(dst, 0); //thrMid8
        sbufWriteU16(dst, 0); //thrExpo8
        sbufWriteU8(dst, 0); //tpa_breakpoint
        sbufWriteU8(dst, 0); //rcExpo
        sbufWriteU8(dst, rcRates[Z]);
        sbufWriteU8(dst, rcRates[Y]);
        sbufWriteU8(dst, 0); //rcExpo

        // added in 1.41
        sbufWriteU8(dst, 0); //throttle_limit_type
        sbufWriteU8(dst, 0); //throttle_limit_percent

        // added in 1.42
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    }
        break;
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, getFcConfig()->ACC_TRIM[PITCH]);
        sbufWriteU16(dst, getFcConfig()->ACC_TRIM[ROLL]);
        sbufWriteU16(dst, getFcConfig()->ACC_TRIM[YAW]);
        break;
    case MSP_FC_VARIANT:
        sbufWriteString(dst, FC_VARIANT);
        break;
    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;
    case MSP_CF_SERIAL_CONFIG: {
        sbufWriteU8(dst, 0); //id
//        enum {
//            MSP = 0,
//            GPS = 1,
//            TELEMETRY_FRSKY = 2,
//            TELEMETRY_HOTT = 3,
//            TELEMETRY_MSP = 4,
//            TELEMETRY_LTM = 4, // LTM replaced MSP
//            TELEMETRY_SMARTPORT = 5,
//            RX_SERIAL = 6,
//            BLACKBOX = 7,
//            TELEMETRY_MAVLINK = 9,
//            ESC_SENSOR = 10,
//            TBS_SMARTAUDIO = 11,
//            TELEMETRY_IBUS = 12,
//            IRC_TRAMP = 13,
//            RUNCAM_DEVICE_CONTROL = 14, // support communitate with RunCam Device
//            LIDAR_TF = 15,
//            FRSKY_OSD = 16
//        };
        uint16_t mask = 0;
        sbufWriteU16(dst, mask | 0b0000000011000001);
        sbufWriteU8(dst, 10);
        sbufWriteU8(dst, 5);
        sbufWriteU8(dst, 5);
        sbufWriteU8(dst, 0);
    }
        break;
    case MSP_BOARD_ALIGNMENT_CONFIG: {
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    }
        break;

    case MSP_RX_MAP: {
        for (int i = 0; i < RX_CHANL_COUNT; i++) {
            sbufWriteU8(dst, i);
        }
    }
        break;
    case MSP_RSSI_CONFIG: {
        sbufWriteU8(dst, 0);
    }
        break;
    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, getFcConfig()->ARM_TIMEOUT_US / 100000); //0,1Sec
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, getFcConfig()->MAX_ARMING_ANGLE);
        break;
    case MSP_RX_CONFIG:
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, getFcConfig()->MAXCHECK);
        sbufWriteU16(dst, getFcConfig()->MIDRC);
        sbufWriteU16(dst, getFcConfig()->MINCHECK);
        sbufWriteU8(dst, 0); //rxConfig()->spektrum_sat_bind);
        sbufWriteU16(dst, 0); //rxConfig()->rx_min_usec);
        sbufWriteU16(dst, 0); // rxConfig()->rx_max_usec);
        sbufWriteU8(dst, 0); // rxConfig()->rcInterpolation);
        sbufWriteU8(dst, 0); // rxConfig()->rcInterpolationInterval);
        sbufWriteU16(dst, 1000); //        sbufWriteU16(dst, rxConfig()->airModeActivateThreshold * 10 + 1000);
        break;
    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, getFcConfig()->ARM_TIMEOUT_US / 100000);
        sbufWriteU8(dst, getFcConfig()->ARM_TIMEOUT_US / 100000);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 1);
        break;
    case MSP_REBOOT: {
        sbufWriteU8(dst, 0); //MSP_REBOOT_FIRMWARE
        rebootFC();
    }
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
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, TARGET_LOOP_HZ); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down
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
        sbufWriteU16(dst, msp_debug_data.data[0]);
        sbufWriteU16(dst, msp_debug_data.data[1]);
        sbufWriteU16(dst, msp_debug_data.data[2]);
        sbufWriteU16(dst, msp_debug_data.data[3]);
        break;
    }
    case MSP_DATAFLASH_SUMMARY:
        break;
    case MSP_DEBUGMSG:
        break;
    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;
    case MSP_FEATURE_CONFIG:
        //FEATURE_RX_MSP
        sbufWriteU32(dst, 1 << 14);
        break;
    case MSP_RAW_IMU: { //some weird scaling for Betaflight configurator
        sensors_t *s = getSonsors();
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf(s->acc.ADCRaw[i] / 2));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf(s->gyro.raw[i] * s->gyro.scale * 4));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, 0);
        }
        break;
    }
    case MSP_MOTOR_CONFIG: {
        config_t* config = getFcConfig();
        sbufWriteU16(dst, config->MINTHROTTLE);
        sbufWriteU16(dst, config->MAXTHROTTLE);
        sbufWriteU16(dst, config->MINCOMMAND);
        break;
    }
    case MSP_ATTITUDE: {
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;
    }
    case MSP_NAME: {
        const int nameLen = strlen(getFcConfig()->PILOTNAME);
        for (int i = 0; i < nameLen; i++) {
            sbufWriteU8(dst, getFcConfig()->PILOTNAME[i]);
        }
    }
        break;
    case MSP_RC: {
        rx_command_t *rx = &getFcControl()->rx;

        for (int i = 0; i < RX_CHANL_COUNT; i++) {
            sbufWriteU16(dst, rx->chan[i]);
        }
    }
        break;
    case MSP_BLACKBOX_START:
        blackboxStart();
        break;
    case MSP_BLACKBOX_STOP:
        blackboxStop();
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
    const unsigned int dataSize = sbufBytesRemaining(src);
//const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_ACC_CALIBRATION:
        if (!getFcStatus()->ARMED) {
            accStartCalibration();
        }
        break;
    case MSP_SET_MOTOR: {
        motors_command_t motors;
        for (int i = 0; i < 4; i++) {
            int16_t read = sbufReadU16(src);
            motors.value[i] = read;
        }
        testMotor(&motors);
    }
        break;
    case MSP_SET_MOTOR_CONFIG: {
        getFcConfig()->MINTHROTTLE = sbufReadU16(src);
        getFcConfig()->MAXTHROTTLE = sbufReadU16(src);
        getFcConfig()->MINCOMMAND = sbufReadU16(src);
    }
        break;
    case MSP_SET_RTC:
        //todo
        break;
    case MSP_SET_ARMING_DISABLED:
        //todo
        break;
    case MSP_SET_RC_TUNING: {
        //TODO
    }
        break;
    case MSP_SET_PID: {
        for (int i = 0; i < 3; i++) {
            getFcConfig()->rate_controller_config.Kp[i] = sbufReadU8(src) / 100.0f;
            getFcConfig()->rate_controller_config.Ki[i] = sbufReadU8(src) / 100.0f;
            getFcConfig()->rate_controller_config.Kd[i] = sbufReadU8(src) / 100.0f;
        }
        getFcConfig()->levelGain = sbufReadU8(src) / 10.0f;
    }
        break;
    case MSP_SET_PID_ADVANCED: {
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src); // was pidProfile.yaw_p_limit
        sbufReadU8(src); // reserved
        sbufReadU8(src); //currentPidProfile->vbatPidCompensation = sbufReadU8(src);
        sbufReadU8(src); //currentPidProfile->feedForwardTransition = sbufReadU8(src);
        sbufReadU8(src); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU16(src); //currentPidProfile->rateAccelLimit = sbufReadU16(src);
        sbufReadU16(src); //currentPidProfile->yawRateAccelLimit = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 2) {
            getFcConfig()->deciLevelAngleLimit = sbufReadU8(src) * 10;
        }
    }
        break;
    case MSP_SET_NAME:
        memset(getFcConfig()->PILOTNAME, 0, 16);
        for (unsigned int i = 0; i < MIN((const unsigned int )15, dataSize); i++) {
            getFcConfig()->PILOTNAME[i] = sbufReadU8(src);
        }
        break;
    case MSP_SET_ARMING_CONFIG:
        sbufReadU8(src); //getFcConfig()->ARM_TIMEOUT_US = sbufReadU8(src) * 100000; //set via fail save
        sbufReadU8(src);
        getFcConfig()->MAX_ARMING_ANGLE = sbufReadU8(src);
        break;
    case MSP_SET_ACC_TRIM:
        getFcConfig()->ACC_TRIM[PITCH] = sbufReadU16(src);
        getFcConfig()->ACC_TRIM[ROLL] = sbufReadU16(src);
        break;
    case MSP_EEPROM_WRITE:
        write_EEPROM(getFcConfig());
        break;
#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG: {
        if (sbufReadU8(src) == BLACKBOX_DEVICE_SERIAL) {
            blackboxConfig()->device = sbufReadU8(src);
            getFcConfig()->blackboxEnabled = true;
        }
    }
        break;
#endif
    case MSP_SET_FAILSAFE_CONFIG:
        getFcConfig()->ARM_TIMEOUT_US = sbufReadU8(src) * 100000;
        break;
    case MSP_SET_RAW_RC: {
        rx_command_t *rx = &getFcControl()->rx;
        uint8_t channelCount = dataSize / sizeof(uint16_t);
        for (int i = 0; i < RX_CHANL_COUNT && i < channelCount; i++) {
            rx->chan[i] = sbufReadU16(src);
        }
        rx->lastReceived = micros();
    }
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
    } else {
        ret = mspProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}
void mspFcProcessReply(mspPacket_t *cmd) {
//no Master so nothing to do here
}
