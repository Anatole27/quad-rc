#include "Sensors.h"

Sensors::Sensors() : imu(), imuInitialized(false)
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

    // IMU initialize at the first run. The IMU has to be read frequently so it is only initialized at the last moment
}

void Sensors::run()
{
    // Initialize IMU/DMP at the first run
    while(!imuInitialized){
        imuInitialized = imu.initialize();
        timeStart = millis();
    }

    // Get data from IMU/DMP
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

void Sensors::resetGyroPath()
{
    imu.resetGyroPath();
    Serial.println(F("Reset IMU"));
}

bool Sensors::endInit()
{
    // initialization is finished when the IMU is horizontal
    if(millis()-timeStart > INIT_TIME)
    {
        resetGyroPath();
        return true;
    }
    else return false;
}
