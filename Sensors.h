#ifndef DEF_SENSORS_H
#define DEF_SENSORS_H

#include "IMU.h"

#define GYRO_THRESHOLD 0.01
#define ANGLE_THRESHOLD 0.05


class Sensors
{
public :
    Sensors();

    void run();
    void getAttitudeState(float* attitudeState);
    void getAttitudeDerivative(float* attitudeDerivative);
    bool endInit();

//private :
    IMU imu;
    float position[3];
    float speed[3];
    float acceleration[3];
    float earthAccel[3];
    float quaternion[4];
    float eulerAngles[3];
    float gyroRates[3];
    bool imuInitialized;
};

extern Sensors *sensors;
#endif
