#include "blackbox/blackbox.h"
#include "fc/tasks.h"
#include "fc/fc.h"
#include "io/motor.h"
#include "io/pin.h"
#include "imu/imu.h"
#include "platform.h"
#include "common/debug.h"
#include "fc/rateController.h"
#include "fc/attitudeController.h"

static control_t *fcControl;
static status_t *fcStatus;
static sensors_t *sensors;
static config_t *fcConfig;

static void debugTask(taskId_e id) {
    taskInfo_t taskInfo;
    getTaskInfo(id, &taskInfo);
//
//    printf("taskInfo: enabled %d  Period: [%d,%d] ", taskInfo.isEnabled, taskInfo.desiredPeriodUs, taskInfo.latestDeltaTimeUs);
//#if defined(USE_TASK_STATISTICS)
//    printf("maxExeTime: %d totalExeTime: %d excTimeAvg:%d PeriodAvg: %d", taskInfo.maxExecutionTimeUs, taskInfo.totalExecutionTimeUs,
//            taskInfo.averageExecutionTimeUs, taskInfo.averageDeltaTimeUs);
//#endif
//    printf("\n");

    printDebug("id: ");
    printInt16Debug((int16_t) id);
    printDebug(" e: ");
    printInt16Debug((int16_t) taskInfo.isEnabled);
    printDebug(" p: [");
    printInt32Debug((int32_t) taskInfo.desiredPeriodUs);
    printDebug(",");
    printInt32Debug((int32_t) taskInfo.latestDeltaTimeUs);
    printDebug("]");
#if defined(USE_TASK_STATISTICS)
    printDebug(" met:");
    printInt32Debug((int32_t) taskInfo.maxExecutionTimeUs);
    printDebug(" aet:");
    printInt32Debug((int32_t) taskInfo.averageExecutionTimeUs);
    printDebug(" pa:");
    printInt32Debug((int32_t) taskInfo.averageDeltaTimeUs);
#endif
    printDebug("\n");
}
static void taskDebugSerial(timeUs_t currentTimeUs) {
    static taskId_e id = 0;
    if (id == 0) {
        printDebug("---Load ");
        printInt16Debug((int16_t) getSystemLoad());
        printDebug("%---\n");
    }
    debugTask(id);
    id++;
    if (id == TASK_COUNT) {
        id = 0;
    }
}

static void taskLed(timeUs_t currentTimeUs) {

    setStatusLedLevel(CALIBRATION_LED, isGyroSensorCalibrationComplete() && accIsCalibrationComplete());

    setStatusLedLevel(ARM_LED, !getStatusLedLevel(ARM_LED));
}

static void taskRx(timeUs_t currentTimeUs) {
    //scale
    fcControl->fc_command.roll = fcControl->rx.chan[ROLL] - fcConfig->MIDRC;
    fcControl->fc_command.pitch = fcControl->rx.chan[PITCH] - fcConfig->MIDRC;
    fcControl->fc_command.yaw = fcControl->rx.chan[YAW] - fcConfig->MIDRC;
    fcControl->fc_command.throttle = fcControl->rx.chan[THROTTLE];

    if (fcControl->rx.chan[AUX1] > 1600) {
        fcStatus->ARMED = true;
    } else {
        fcStatus->ARMED = false;
    }

}
static void taskHandleSerial(timeUs_t currentTimeUs) {
    processMSP();
}

static void taskLoop(timeUs_t currentTimeUs) {
    //Timeout 500ms
    if (cmpTimeUs(currentTimeUs, fcControl->rx.lastReceived) > fcConfig->ARM_TIMEOUT_US) {
        fcStatus->ARMED = false;
        resetRx();
        setStatusLedLevel(ERROR_LED, true);
    } else {
        setStatusLedLevel(ERROR_LED, false);
    }

    //gyro based operations
    sensors->gyro.readFn(&sensors->gyro);
    updateGyro(currentTimeUs);

    if (!fcStatus->ARMED) {
        resetRateController();
        fcControl->mixer_command.axis[X] = 0;
        fcControl->mixer_command.axis[Y] = 0;
        fcControl->mixer_command.axis[Z] = 0;
    } else {
        //update rate controller make sure sensors->gyro.filtered[.] gets the new gyro data
        updateRateController(fcControl, &sensors->gyro, &fcConfig->rate_controller_config, currentTimeUs); //TODO set Arming back
    }

    //update Motor
    //printf("pid motor commands = [%d,%d,%d;%d]", motor_command->value[ROLL], motor_command->value[PITCH], motor_command->value[YAW], motor_command->value[THROTTLE]);
    updateMotors();
#ifdef USE_BLACKBOX
    //init blackBox
    blackboxUpdate(currentTimeUs);
#endif

}
static void taskAttitude(timeUs_t currentTimeUs) {
    sensors->acc.readFn(&sensors->acc);

    updateACC(currentTimeUs);

    // update attitude make sure data in gyro.data[.] is in degree/s and acc.data[.] is in G
    updateEstimatedAttitude(currentTimeUs);

    if (fcStatus->ANGLE) {
        // attitude controller
        fcControl->attitude_command.axis[ROLL] = fcControl->fc_command.roll;
        fcControl->attitude_command.axis[PITCH] = fcControl->fc_command.pitch;
        fcControl->attitude_command.axis[YAW] = fcControl->fc_command.yaw;
        //printf("%d %d,%d,%d  %d\n", fcStatus->ARMED, fcControl->attitude_command.axis[ROLL], fcControl->attitude_command.axis[PITCH], fcControl->attitude_command.axis[YAW], fcControl->fc_command.throttle);
        updateAttitudeController(fcControl, fcConfig, currentTimeUs, &attitude);
    } else {
        fcControl->rate_command.axis[ROLL] = fcControl->fc_command.roll;
        fcControl->rate_command.axis[PITCH] = fcControl->fc_command.pitch;
        fcControl->rate_command.axis[YAW] = fcControl->fc_command.yaw;
    }

}

static task_t tasks[TASK_COUNT] = {
    [TASK_DEBUG] = {
        .taskName = "TASK_DEBUG",
        .taskFunc = taskDebugSerial,
        .staticPriority = 0,
        .desiredPeriodUs = TASK_PERIOD_HZ(4), },
    [TASK_SYSTEM] = {
        .taskName = "TASK_SYSTEM",
        .taskFunc = taskSystemLoad,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(10), },
    [TASK_ATTITUDE] = {
        .taskName = "TASK_ATTITUDE",
        .taskFunc = taskAttitude,
        .staticPriority = 2,
        .desiredPeriodUs = TASK_PERIOD_HZ(250), },
    [TASK_RX] = {
        .taskName = "TASK_RX",
        .taskFunc = taskRx,
        .staticPriority = 3,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), },
    [TASK_LED] = {
        .taskName = "TASK_LED",
        .taskFunc = taskLed,
        .staticPriority = 0,
        .desiredPeriodUs = TASK_PERIOD_HZ(5), },
    [TASK_SERIAL] = {
        .taskName = "TASK_SERIAL",
        .taskFunc = taskHandleSerial,
        .staticPriority = 4,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), },
    [TASK_LOOP] = {
        .taskName = "TASK_LOOP",
        .taskFunc = taskLoop,
        .staticPriority = 200,
        .desiredPeriodUs = TASK_PERIOD_HZ(TARGET_LOOP_HZ), } };

/**
 * get the task with the given taskId or NULL on error
 * @param taskId
 * @return
 */
task_t *getTask(unsigned taskId) {
    return &tasks[taskId];
}

void tasksInit(void) {
    //set global variable which are often used bye tasks
    fcControl = getFcControl();
    fcStatus = getFcStatus();
    sensors = getSonsors();
    fcConfig = getFcConfig();
    schedulerInit();
    //enebale tasks
    //setTaskEnabled(TASK_DEBUG, true);
    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_LOOP, true);
    setTaskEnabled(TASK_ATTITUDE, true);
    setTaskEnabled(TASK_RX, true);
    setTaskEnabled(TASK_LED, true);
}
