#ifndef DEF_SENSORS
#define DEF_SENSORS

#include "Util.h"

class Sensors
{
public :
    Sensors();

    void getAttitudeState(float* attitudeState);
    void getAttitudeDerivative(float* attitudeDerivative);

private :
    IMU imu;
};

extern Sensors *sensors;
#endif
