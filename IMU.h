#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "MatrixMath.h"



class IMU
{
public:
    IMU();
    bool initialize();
    void readData();
    void getQuaternion(float* quat);
    void getEulerAngles(float* eulerAngles); // phi,theta,psi
    void getGyro(float* gyroRates);
    void getRelativeAcceleration(float* relativeAccel);
    void getLinearAcceleration(float* acceleration);
    void getEarthAcceleration(float* earthAccel);
    void resetGyroPath();

private :
    MPU6050 mpu;
    bool dmpReady; // set true if DMP init was successful
    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount; // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    // orientation/motion vars
    Quaternion q; // [w, x, y, z] quaternion container
    Quaternion qOffset; // quaternion offset for reset path
    VectorInt16 aa; // [x, y, z] accel sensor measurements
    VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z] gravity vector
    float euler[3]; // [psi, theta, phi] Euler angle container
    int16_t gyro[3]; //[P,Q,R] angularSpeeds

};

#endif // IMU_H
