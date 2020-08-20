#include "imu/imu.h"
#include "imu/betaflightIMU.h"

attitude_estimator_t attitude_estimator;
// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
attitudeEulerAngles_t attitude = EULER_INITIALIZE;


void initImu(void) {
    attitude_estimator.update = &updatebetaIMU;
    betaIMUInit();
}
void updateEstimatedAttitude(timeUs_t currentTime) {
    attitude_estimator.update(currentTime);
}
