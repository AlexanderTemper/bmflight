#pragma once

#include "global.h"
#include "common/time.h"

#define USE_TASK_STATISTICS

#if defined(USE_TASK_STATISTICS)
#define TASK_STATS_MOVING_SUM_COUNT 32
#endif

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_SERIAL,
    TASK_DEBUG,
    /* Count of real tasks */
    TASK_COUNT,
} taskId_e;

typedef struct {
    bool isEnabled;
    timeDelta_t desiredPeriodUs;
    timeDelta_t latestDeltaTimeUs;
#if defined(USE_TASK_STATISTICS)
    timeUs_t maxExecutionTimeUs;
    timeUs_t totalExecutionTimeUs;
    timeUs_t averageExecutionTimeUs;
    timeUs_t averageDeltaTimeUs;
#endif
} taskInfo_t;

typedef struct {
#if defined(USE_TASK_STATISTICS)
    timeUs_t movingSumExecutionTimeUs;  // moving sum over 32 samples
    timeUs_t movingSumDeltaTimeUs;// moving sum over 32 samples
    timeUs_t maxExecutionTimeUs;
    timeUs_t totalExecutionTimeUs;// total time consumed by task since boot
#endif
    const char * taskName;
    void (*taskFunc)(timeUs_t currentTimeUs);
    timeDelta_t desiredPeriodUs;      // target period of execution
    timeDelta_t taskLatestDeltaTimeUs;
    timeUs_t lastExecutedAtUs;        // last time of invocation
    timeUs_t lastSignaledAtUs;        // time of invocation event for event-driven tasks
    timeUs_t lastDesiredAt;         // time of last desired execution
} task_t;

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo);
void setTaskEnabled(taskId_e taskId, bool enabled);
void schedulerInit(void);
void scheduler(void);

void taskSystemLoad(timeUs_t currentTimeUs);
