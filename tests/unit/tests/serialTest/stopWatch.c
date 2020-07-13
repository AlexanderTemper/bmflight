#include <stdio.h>
#include "stopWatch.h"

void startTimer(stopWatch_t *w) {
    gettimeofday(&w->begin, (struct timezone *) 0);
    w->running = true;
}

long getTimeUsec(stopWatch_t *w) {
    struct timeval now;
    gettimeofday(&now, (struct timezone *) 0);

    long seconds = now.tv_sec - w->begin.tv_sec;
    long useconds = now.tv_usec - w->begin.tv_usec;
    if (useconds < 0) {
        useconds += 1000000;
        seconds--;
    }
    return useconds;
}
