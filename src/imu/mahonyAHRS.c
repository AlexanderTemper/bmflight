#include <math.h>
//#include <stdio.h>

#include "imu/mahonyAHRS.h"
#include "imu/imu.h"
#include "sensor/sensor.h"

#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
static void imuMahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {

}

static void getEuler(void) {

    attitutde_r = 100;
    attitutde_p = -200;
    attitutde_y = 300;

//    float roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
//    float pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
//    float yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));

    if (attitutde_y < 0) {
        attitutde_y += 3600;
    }
    //printf("att: r: %f, p: %f, y: %f\n", roll, pitch, yaw);
}

void updateMahony(timeUs_t currentTime) {
    static timeUs_t lastExecute = 0;
    const timeDelta_t timeDiff = cmpTimeUs(currentTime, lastExecute);
    if (0 == timeDiff) {
       // printf("timeDiff 0\n");
        return;
    }

    sensors_t *sensors = getSonsors();
    float gx = DEGREES_TO_RADIANS(sensors->gyro.data[X]);
    float gy = DEGREES_TO_RADIANS(sensors->gyro.data[Y]);
    float gz = DEGREES_TO_RADIANS(sensors->gyro.data[Z]);

    //printf("update Mahony %f \n", timeDiff/(float)1000);
    lastExecute = currentTime;
    imuMahonyAHRSupdate(gx, gy, gz, sensors->acc.data[X], sensors->acc.data[Y], sensors->acc.data[Z], timeDiff * 1e-6f);
    getEuler();
}

