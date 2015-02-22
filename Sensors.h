#ifndef DEF_SENSORS_H
#define DEF_SENSORS_H

#include "IMU.h"

class Sensors
{
public :
    Sensors();

    void run();
    void getAttitudeState(float* attitudeState);
    void getAttitudeDerivative(float* attitudeDerivative);

//private :
    IMU imu;
    float position[3];
    float speed[3];
    float acceleration[3];
    float earthAccel[3];
    float quaternion[4];
    float eulerAngles[3];
    float gyroRates[3];
};

extern Sensors *sensors;
#endif
