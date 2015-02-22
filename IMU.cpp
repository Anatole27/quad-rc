#include "IMU.h"

IMU::IMU() : mpu(0x69), dmpReady(false), mpuIntStatus(0), devStatus(0),
    fifoCount(0), q(1,0,0,0), aa(0,0,0), aaReal(0,0,0), aaWorld(0,0,0), gravity(0,0,0)
{
    for(int i = 0;i<3;i++){
        euler[i] = 0;
        gyro[i] = 0;
    }
    for(int i = 0;i<64;i++) fifoBuffer[i] = 0;
}

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
// Method called with interrupt pin 2
void dmpDataReady() {
    mpuInterrupt = true;
}

bool IMU::initialize()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, &dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        return true; // Initializing completed
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false; // Initialization went wrong
    }
}


// Read the data from DMP
void IMU::readData() {
    // Empty the buffer to avoid overflow
    Serial.println(mpuInterrupt);
    while(mpuInterrupt || fifoCount >= packetSize)
    {
        //reset interrupt
        mpuInterrupt = false;
        Serial.println(millis());
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;


            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);

            //get EulerAngles phi theta psi
            mpu.dmpGetEuler(euler, &q);

            //get Gyro rates
            mpu.dmpGetGyro(gyro, fifoBuffer);

            // get Gravity vector in body axis
            mpu.dmpGetGravity(&gravity, &q);

            // get relative acceleration measured
            mpu.dmpGetAccel(&aa,fifoBuffer);

            // get real acceleration in body axis
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

            // get acceleration in earth axis
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        }
    }
}

void IMU::getQuaternion(float *quat)
{
        quat[0] = q.w;
        quat[1] = q.x;
        quat[2] = q.y;
        quat[3] = q.z;
}

void IMU::getEulerAngles(float* eulerAngles)
{
    Matrix.Copy((float*)euler,3,1,eulerAngles);
}

void IMU::getGyro(float* gyroRates)
{
    gyroRates[0] = gyro[0];
    gyroRates[1] = gyro[1];
    gyroRates[2] = gyro[2];
}

void IMU::getRelativeAcceleration(float* relativeAccel)
{
    relativeAccel[0] = aa.x;
    relativeAccel[1] = aa.y;
    relativeAccel[2] = aa.z;
}

void IMU::getLinearAcceleration(float *acceleration)
{
    acceleration[0] = aaReal.x;
    acceleration[1] = aaReal.y;
    acceleration[2] = aaReal.z;
}

void IMU::getEarthAcceleration(float *earthAccel)
{
    earthAccel[0] = aaWorld.x;
    earthAccel[1] = aaWorld.y;
    earthAccel[2] = aaWorld.z;
}
