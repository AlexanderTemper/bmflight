#include <math.h>
//#include <stdio.h>

#include "common/maths.h"
#include "imu/mahonyAHRS.h"
#include "imu/imu.h"

#include "sensor/sensor.h"

void mahonyInit(void) {

}

void updateMahony(timeUs_t currentTimeUs) {
    static timeUs_t previousIMUUpdateTime;
    const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

    if (deltaT <=0 ) {
        // printf("timeDiff 0\n");
        return;
    }

    float gyroAverage[3];
//        gyroGetAccumulationAverage(gyroAverage);
//
//        if (accGetAccumulationAverage(accAverage)) {
//            useAcc = imuIsAccelerometerHealthy(accAverage);
//        }

    sensors_t *sensors = getSonsors();

    gyroAverage[X] = sensors->gyro.data[X];
    gyroAverage[Y] = sensors->gyro.data[Y];
    gyroAverage[Z] = sensors->gyro.data[Z];

    //printf("update Mahony %f \n", timeDiff/(float)1000);
}

