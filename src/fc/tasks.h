#pragma once

#include "global.h"
#include "scheduler/scheduler.h"



void tasksInit(void);
/**
 * get the task with the given taskId or NULL on error
 * @param taskId
 * @return
 */
task_t *getTask(unsigned taskId);
