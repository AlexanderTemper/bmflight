#include "imu/imu.h"

int16_t attitutde_r;
int16_t attitutde_p;
int16_t attitutde_y;

static attitude_estimator_t *estimator;

void initImu(attitude_estimator_t *est) {
    estimator = est;
}
void updateEstimatedAttitude(timeUs_t currentTime) {
    estimator->update(currentTime);
}
