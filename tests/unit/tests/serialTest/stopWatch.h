#pragma once
#include <stdbool.h>
#include <sys/time.h>

typedef struct stopWatch_s {
    long seconds;
    long useconds;
    struct timeval begin;
    bool running;
} stopWatch_t;

void startTimer(stopWatch_t *w);

long getTimeUsec(stopWatch_t *w);
