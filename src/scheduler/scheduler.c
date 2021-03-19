#include "scheduler/scheduler.h"
#include "fc/tasks.h"
#include <string.h>

static timeUs_t schedulerTotalTimeUs = 0;
static timeUs_t schedulerWorkTimeUs = 0;
static uint16_t systemLoad = 0;
static int taskQueuePos = 0;
static int taskQueueSize = 0;
static task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

//#include <stdio.h> //TODO WEG !!!

static void queueClear(void) {
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

static bool queueContains(task_t *task) {
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

static task_t *queueFirst(void) {
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

static task_t *queueNext(void) {
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

static bool queueAdd(task_t *task) {
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii + 1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }

    return false;
}

static bool queueRemove(task_t *task) {
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii + 1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

void setTaskEnabled(taskId_e taskId, bool enabled) {
    if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

void schedulerInit(void) {
    queueClear();
    queueAdd(getTask(TASK_SYSTEM)); //Add System Task
}

static timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs) {
    timeUs_t taskExecutionTimeUs = 0;
    if (selectedTask) {
        selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastExecutedAtUs);
        selectedTask->lastExecutedAtUs = currentTimeUs;
#if defined(USE_TASK_STATISTICS)
        const timeUs_t currentTimeBeforeTaskCallUs = micros();
#endif
        selectedTask->taskFunc(currentTimeUs);
#if defined(USE_TASK_STATISTICS)
        taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
        selectedTask->movingSumExecutionTimeUs += taskExecutionTimeUs - (selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT);
        selectedTask->movingSumDeltaTimeUs += selectedTask->taskLatestDeltaTimeUs - (selectedTask->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT);
        selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
        selectedTask->maxExecutionTimeUs = (selectedTask->maxExecutionTimeUs > taskExecutionTimeUs) ? selectedTask->maxExecutionTimeUs : taskExecutionTimeUs;
#endif
    }

    return taskExecutionTimeUs;
}

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo) {
    task_t *task = getTask(taskId);
    taskInfo->isEnabled = queueContains(task);
    taskInfo->desiredPeriodUs = task->desiredPeriodUs;
    taskInfo->latestDeltaTimeUs = task->taskLatestDeltaTimeUs;
#if defined(USE_TASK_STATISTICS)
    taskInfo->maxExecutionTimeUs = task->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = task->totalExecutionTimeUs;
    taskInfo->averageExecutionTimeUs = task->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTimeUs = task->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
#endif
}

task_t* getTaskQueueAt(uint8_t pos) {
    return taskQueueArray[pos];
}
#define GYRO_TASK_GUARD_INTERVAL_US 400

static void TimeSliceScedular(void){
    static bool attPeriod = true;
    const timeUs_t schedulerStartTimeUs = micros();
        timeUs_t currentTimeUs = schedulerStartTimeUs;
        task_t *selectedTask = NULL;
        //printf("start was %d \t", currentTimeUs);

        task_t* loopTask = getTask(TASK_LOOP);
        const timeUs_t nextLoopTaskTime = loopTask->lastExecutedAtUs + loopTask->desiredPeriodUs;
        const timeDelta_t loopTaskDelayUs = cmpTimeUs(nextLoopTaskTime, currentTimeUs);  // time until the next expected gyro sample


        if (loopTaskDelayUs <= 0) { //select gyro Task
            schedulerExecuteTask(loopTask, currentTimeUs);
            if(attPeriod){
                schedulerExecuteTask(getTask(TASK_ATTITUDE), currentTimeUs);
            } else {
                schedulerExecuteTask(getTask(TASK_SERIAL), currentTimeUs);
                schedulerExecuteTask(getTask(TASK_RX), currentTimeUs);
                //execute all other Task if needed
                for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
                    // Select Task if time is expired
                    currentTimeUs = micros();
                    if (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) >= task->desiredPeriodUs) {
                        schedulerExecuteTask(task, currentTimeUs);
                    }
                }
            }
            attPeriod = !attPeriod;

        }
}
void scheduler(void) {
    //return TimeSliceScedular();
    const timeUs_t schedulerStartTimeUs = micros();
    timeUs_t currentTimeUs = schedulerStartTimeUs;
    task_t *selectedTask = NULL;
    //printf("start was %d \t", currentTimeUs);

    task_t* loopTask = getTask(TASK_LOOP);
    const timeUs_t nextLoopTaskTime = loopTask->lastExecutedAtUs + loopTask->desiredPeriodUs;
    const timeDelta_t loopTaskDelayUs = cmpTimeUs(nextLoopTaskTime, currentTimeUs);  // time until the next expected gyro sample
    if (loopTaskDelayUs <= 0) { //select gyro Task
        schedulerExecuteTask(loopTask, currentTimeUs);
    } else if (loopTaskDelayUs > GYRO_TASK_GUARD_INTERVAL_US) { //only dispatch an task if gyro task has enough time left
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
            // Select Task if time is expired
            currentTimeUs = micros();
            if(task->taskFunc == getTask(TASK_ATTITUDE)->taskFunc && loopTaskDelayUs < 1000){ //Prevent Running the att task if not enogth time left
                continue;
            }
            if (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) >= task->desiredPeriodUs) {
                selectedTask = task;
                break;
            }
        }
    }

    if (selectedTask) {
        schedulerExecuteTask(selectedTask, currentTimeUs);
        schedulerWorkTimeUs += cmpTimeUs(micros(), schedulerStartTimeUs);
    }
    schedulerTotalTimeUs += cmpTimeUs(micros(), schedulerStartTimeUs);

}
uint16_t getSystemLoad(void) {
    return systemLoad;
}
void taskSystemLoad(timeUs_t currentTimeUs) {
    // Calculate system load
    systemLoad = (100 * schedulerWorkTimeUs) / (schedulerTotalTimeUs);
    schedulerWorkTimeUs = 0;
    schedulerTotalTimeUs = 0;
}
