#pragma once
#define MSP_PROTOCOL_VERSION       0

#define API_VERSION_MAJOR          1  // increment when major changes are made
#define API_VERSION_MINOR          42 // increment after a release, to set the version for all changes to go into the following release (if no changes to MSP are made between the releases, this can be reverted before the release)

#define MSP_API_VERSION            1
#define MSP_FC_VARIANT             2
#define MSP_FC_VERSION             3
#define MSP_BOARD_INFO             4
#define MSP_BUILD_INFO             5
#define MSP_NAME                   10      //out message          Returns user set board name - betaflight
#define MSP_SET_NAME               11   //in message           Sets board name - betaflight
#define MSP_BATTERY_CONFIG         32
#define MSP_FEATURE_CONFIG         36
#define MSP_BOARD_ALIGNMENT_CONFIG 38
#define MSP_MIXER_CONFIG           42
#define MSP_RX_CONFIG              44
#define MSP_RSSI_CONFIG            50
#define MSP_CF_SERIAL_CONFIG       54
#define MSP_PID_CONTROLLER         59
#define MSP_ARMING_CONFIG          61
#define MSP_SET_ARMING_CONFIG      62
#define MSP_RX_MAP                 64     //out message get channel map (also returns number of channels total)
#define MSP_REBOOT                 68     //in message reboot settings
#define MSP_DATAFLASH_SUMMARY      70     //out message - get description of dataflash chip
#define MSP_FAILSAFE_CONFIG        75     //out message         Returns FC Fail-Safe settings
#define MSP_SET_FAILSAFE_CONFIG    76     //in message          Sets FC Fail-Safe settings
#define MSP_BLACKBOX_CONFIG        80     //out message         Get blackbox settings
#define MSP_SET_BLACKBOX_CONFIG    81     //in message          Set blackbox settings
#define MSP_ADVANCED_CONFIG        90
#define MSP_PID_ADVANCED           94
#define MSP_SET_PID_ADVANCED       95
#define MSP_SET_ARMING_DISABLED    99
#define MSP_STATUS                 101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU                102
#define MSP_MOTOR                  104    //out message         motors
#define MSP_RC                     105    //out message         rc channels and more
#define MSP_ATTITUDE               108    //out message         2 angles 1 heading
#define MSP_ANALOG                 110
#define MSP_RC_TUNING              111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                    112    //out message         P I D coeff (9 are used currently)
#define MSP_BOXNAMES               116    //out message         the aux switch names
#define MSP_PIDNAMES               117    //out message         the PID names
#define MSP_RC_DEADBAND            125    //out message         deadbands for yaw alt pitch roll
#define MSP_MOTOR_CONFIG           131    //out message         Motor configuration (min/max throttle, etc)
#define MSP_STATUS_EX              150    //out message         cycletime, errors_count, CPU load, sensor present etc
#define MSP_UID                    160
#define MSP_SET_RAW_RC             200
#define MSP_SET_PID                202    //in message          P I D coeff (9 are used currently
#define MSP_SET_RC_TUNING          204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
#define MSP_ACC_CALIBRATION        205
#define MSP_SET_MOTOR              214    //in message          PropBalance function
#define MSP_SET_MOTOR_CONFIG       222    //out message         Motor configuration (min/max throttle, etc)
#define MSP_SET_ACC_TRIM           239    //in message          set acc angle trim values
#define MSP_ACC_TRIM               240    //out message         get acc angle trim values
#define MSP_SET_RTC                246    //in message          Sets the RTC clock
#define MSP_EEPROM_WRITE           250
#define MSP_DEBUG                  254


// Additional commands that are not compatible with MultiWii
#define MSP_DEBUGMSG               253





