#include "imu/imu.h"

static attitude_estimator_t *estimator;

void initImu(attitude_estimator_t *est) {
    estimator = est;
    est->init();
}
void updateEstimatedAttitude(timeUs_t currentTime) {
    estimator->update(currentTime);
}
