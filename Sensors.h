#ifndef DEF_SENSORS_H
#define DEF_SENSORS_H

#include "IMU.h"
#include "Arduino.h"

#define GYRO_THRESHOLD 0.01
#define ANGLE_THRESHOLD 0.05
#define INIT_TIME 20000


class Sensors
{
public :
    Sensors();

    void run();
    void getAttitudeState(float* attitudeState);
    void getAttitudeDerivative(float* attitudeDerivative);
    bool endInit();
    void resetGyroPath();

//private :
    IMU imu;
    float position[3];
    float speed[3];
    float acceleration[3];
    float earthAccel[3];
    float quaternion[4];
    float eulerAngles[3]; // phi, theta, psi
    float gyroRates[3]; // p,q,r
    bool imuInitialized;
    long timeStart;
};

extern Sensors *sensors;
#endif
