#include "fc/tasks.h"

#include "common/debug.h"
#include "platform.h"
//#include <stdio.h> //TODO WEG !!!

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
    if(id == 0){
        printDebug("---task---\n");
    }
    debugTask(id);
    id++;
    if(id == TASK_COUNT){
        id = 0;
    }
}

static void taskHandleSerial(timeUs_t currentTimeUs)
{
    processMSP();
}

static task_t tasks[TASK_COUNT] = {
        [TASK_SYSTEM] = {
                .taskName = "TASK_SYSTEM",
            .taskFunc = taskSystemLoad,
            .staticPriority = 0,
            .desiredPeriodUs = TASK_PERIOD_HZ(10),
        },
        [TASK_SERIAL] = {
            .taskName = "TASK_SERIAL",
            .taskFunc = taskHandleSerial,
            .staticPriority = 1,
            .desiredPeriodUs = TASK_PERIOD_HZ(100),
        },
        [TASK_DEBUG] = {
            .taskName = "TASK_DEBUG",
            .taskFunc = taskDebugSerial,
            .staticPriority = 0,
            .desiredPeriodUs = TASK_PERIOD_HZ(4),
        },
};



/**
 * get the task with the given taskId or NULL on error
 * @param taskId
 * @return
 */
task_t *getTask(unsigned taskId){
    return &tasks[taskId];
}

void tasksInit(void) {
    schedulerInit();
    setTaskEnabled(TASK_DEBUG, true);
    setTaskEnabled(TASK_SERIAL, true);
}
