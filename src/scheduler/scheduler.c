#include "scheduler/scheduler.h"
#include "fc/tasks.h"
#include <string.h>

static uint32_t totalWaitingTasksSamples;
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

static bool queueAdd(task_t *task) { //TODO add Priority
    if ((taskQueueSize >= TASK_COUNT)) {
        return false;
    }
    taskQueueArray[taskQueueSize] = task;
    taskQueueSize++;
    return true;
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

void scheduler(void) {
    const timeUs_t schedulerStartTimeUs = micros();
    timeUs_t currentTimeUs = schedulerStartTimeUs;
    timeUs_t taskExecutionTimeUs = 0;
    timeDelta_t taskRequiredTimeUs = 0;
    task_t *selectedTask = NULL;
    //printf("start was %d \t", currentTimeUs);

    for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
        // Select Task if time is expired
        if (cmpTimeUs(micros(), task->lastExecutedAtUs) >= task->desiredPeriodUs) {
            selectedTask = task;
            break;
        }
    }

    totalWaitingTasksSamples++;

    if (selectedTask) {
        //printf("select task %s\n",selectedTask->taskName);
        // Add in the time spent so far in check functions and the scheduler logic
        taskRequiredTimeUs += cmpTimeUs(micros(), currentTimeUs);
        // start Task
        schedulerExecuteTask(selectedTask, currentTimeUs);
        taskRequiredTimeUs += cmpTimeUs(micros(), currentTimeUs);
    }
}

void taskSystemLoad(timeUs_t currentTimeUs) {//TODO
    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        //printf("taskSystemLoad: done %d nothing \n", totalWaitingTasksSamples);
        totalWaitingTasksSamples = 0;
    }
}
