#include "sput.h"
#include "common/debug.h"
#include "scheduler/scheduler.h"
#include "common/time.h"

static void task1(timeUs_t currentTimeUs) {

}

static void task2(timeUs_t currentTimeUs) {

}

static void task3(timeUs_t currentTimeUs) {

}

static task_t tasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "TASK_SYSTEM",
        .taskFunc = task1,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(10), },
    [TASK_SERIAL] = {
        .taskName = "TASK_SERIAL",
        .taskFunc = task2,
        .staticPriority = 2,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), },
    [TASK_DEBUG] = {
        .taskName = "TASK_DEBUG",
        .taskFunc = task3,
        .staticPriority = 3,
        .desiredPeriodUs = TASK_PERIOD_HZ(4), }, };

task_t *getTask(unsigned taskId);
task_t *getTask(unsigned taskId) {
    return &tasks[taskId];
}

timeMs_t microstep = 0;

static timeMs_t micros_imp(void) {
    return microstep;
}
static timeMs_t milli_imp(void) {
    return 1000 * micros_imp();
}
static void dummy(timeUs_t nsec) {

}

static task_t* taskQueueArray[TASK_COUNT + 1];

static void copyTask(void) {
    for (int i = 0; i < TASK_COUNT; i++) {
        taskQueueArray[i] = getTaskQueueAt(i);
    }
}
static void test_scheduler_add(void) {

    setTaskEnabled(TASK_SYSTEM, true);
    copyTask();
    sput_fail_unless(taskQueueArray[0]->taskFunc == task1, "1 Task");

    setTaskEnabled(TASK_SERIAL, true);
    copyTask();
    sput_fail_unless(taskQueueArray[0]->taskFunc == task2, "2 Task");
    sput_fail_unless(taskQueueArray[1]->taskFunc == task1, "1 Task");

    setTaskEnabled(TASK_DEBUG, true);
    copyTask();
    sput_fail_unless(taskQueueArray[0]->taskFunc == task3, "3 Task");
    sput_fail_unless(taskQueueArray[1]->taskFunc == task2, "2 Task");
    sput_fail_unless(taskQueueArray[2]->taskFunc == task1, "1 Task");

    setTaskEnabled(TASK_SERIAL, true);
    copyTask();
    sput_fail_unless(taskQueueArray[0]->taskFunc == task3, "3 Task");
    sput_fail_unless(taskQueueArray[1]->taskFunc == task2, "2 Task");
    sput_fail_unless(taskQueueArray[2]->taskFunc == task1, "1 Task");

    setTaskEnabled(TASK_SERIAL, false);
    copyTask();
    sput_fail_unless(taskQueueArray[0]->taskFunc == task3, "3 Task");
    sput_fail_unless(taskQueueArray[1]->taskFunc == task1, "2 Task");
    sput_fail_unless(taskQueueArray[2] == NULL, "1 Task");
}

int main(int argc, char *argv[]) {
    sput_start_testing()
    ;
    initTime(&milli_imp, &micros_imp, &dummy);

    sput_enter_suite("Test scheduler");
    sput_run_test(test_scheduler_add);
    sput_finish_testing()
    ;

    return sput_get_return_value();
}
