#pragma once
#define MSP_PROTOCOL_VERSION       0

#define API_VERSION_MAJOR          1  // increment when major changes are made
#define API_VERSION_MINOR          42 // increment after a release, to set the version for all changes to go into the following release (if no changes to MSP are made between the releases, this can be reverted before the release)

#define MSP_API_VERSION           1
#define MSP_FC_VARIANT            2
#define MSP_FC_VERSION            3
#define MSP_BOARD_INFO            4
#define MSP_BUILD_INFO            5
#define MSP_NAME                  10      //out message          Returns user set board name - betaflight
#define MSP_BATTERY_CONFIG        32
#define MSP_FEATURE_CONFIG        36
#define MSP_MIXER_CONFIG          42
#define MSP_DATAFLASH_SUMMARY     70     //out message - get description of dataflash chip
#define MSP_SET_ARMING_DISABLED   99
#define MSP_STATUS                101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU               102
#define MSP_MOTOR                 104    //out message         motors
#define MSP_ATTITUDE              108    //out message         2 angles 1 heading
#define MSP_ANALOG                110
#define MSP_BOXNAMES              116    //out message         the aux switch names
#define MSP_MOTOR_CONFIG          131    //out message         Motor configuration (min/max throttle, etc)
#define MSP_STATUS_EX             150    //out message         cycletime, errors_count, CPU load, sensor present etc
#define MSP_UID                   160
#define MSP_SET_RAW_RC            200
#define MSP_SET_MOTOR             214    //in message          PropBalance function
#define MSP_ACC_TRIM              240    //out message         get acc angle trim values
#define MSP_SET_RTC               246    //in message          Sets the RTC clock
#define MSP_DEBUG                 254


// Additional commands that are not compatible with MultiWii
#define MSP_DEBUGMSG              253





