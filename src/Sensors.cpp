#include "Sensors.h"

Sensors::Sensors() : imu(), imuInitialized(false), alpha_angle(0), alpha_gyro(ALPHA_INIT), timeIntermediate(0)
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
    float meas[6] = {0};
    imu.readData();
    imu.getEulerAngles((float*)meas);
    imu.getGyro((float*)meas + 3);

    // alpha Filter
    float state[6] = {0};
    getEulerAngles(state);
    getGyroRates(state + 3);
    // state = alpha*state + (1-alpha)*meas; alpha few with few noise
    Matrix.Scale((float*)state , 3 , 1 , alpha_angle);
    Matrix.Scale((float*)state+3 , 3 , 1 , alpha_gyro);
    Matrix.Scale((float*)meas , 3 , 1 , 1-alpha_angle);
    Matrix.Scale((float*)meas+3 , 3 , 1 , 1-alpha_gyro);
    Matrix.Add((float*)state,(float*)meas,6,1,(float*)state);

    Matrix.Copy((float*)state,3,1,(float*)eulerAngles);
    Matrix.Copy((float*)state+3,3,1,(float*)gyroRates);
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

void Sensors::getEulerAngles(float* euler)
{
    Matrix.Copy(eulerAngles,3,1,euler);
}

void Sensors::getGyroRates(float* gyro)
{
    Matrix.Copy(gyroRates,3,1,gyro);
}

void Sensors::resetGyroPath()
{
    // IMU reset
    imu.resetGyroPath();

    Serial.println(F("Reset IMU"));
}

bool Sensors::endInit()
{
    // initialization is finished when the IMU is horizontal
    if(millis()-timeStart > INIT_TIME)
    {
        Serial.println(F("Reset gyro path"));
        resetGyroPath();
        return true;
    }
    else
    {
      if(millis() - timeIntermediate > 1000)
      {
          Serial.print(F("Remaining time = "));
          Serial.println((INIT_TIME - millis() + timeStart)/1000);
          timeIntermediate = millis();
      }
    }return false;
}
