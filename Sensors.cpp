#include "Sensors.h"

Sensors::Sensors() : imu()
{
    // Initialize variables at 0
    for(int i = 0;i<3;i++){
        position[i] = 0;
        speed[i] = 0;
        acceleration[i] = 0;
        earthAccel[i] = 0;
        eulerAngles[i] = 0;
        gyroRates[i] = 0;
    }

    for(int i = 0;i<4;i++){
        quaternion[i] = 0;
    }

    // Initialize IMU/DMP
    bool imuInitialized = false;
    while(!imuInitialized)
        imuInitialized = imu.initialize();
}

void Sensors::run()
{// Kalman to implement
    // Position to update
    // speed to update
    imu.readData();
    imu.getLinearAcceleration((float*)acceleration);
    imu.getEarthAcceleration((float*)earthAccel);
    imu.getEulerAngles((float*)eulerAngles);
    imu.getGyro((float*)gyroRates);
    imu.getQuaternion((float*)quaternion);
}

void Sensors::getAttitudeState(float* attitudeState)
{
    attitudeState[0] = position[2];
    attitudeState[1] = eulerAngles[0];
    attitudeState[2] = eulerAngles[1];
    attitudeState[3] = eulerAngles[2];
}


void Sensors::getAttitudeDerivative(float* attitudeDerivative)
{
    attitudeDerivative[0] = speed[2];
    attitudeDerivative[1] = gyroRates[0];
    attitudeDerivative[2] = gyroRates[1];
    attitudeDerivative[3] = gyroRates[2];
}
