#include "IMU.h"
#include "MPU6050_6Axis_MotionApps20.h"

IMU::IMU() : mpu(0x69), dmpReady(false), mpuIntStatus(0), devStatus(0),
    fifoCount(0), q(1,0,0,0), qOffset(1,0,0,0), aa(0,0,0), aaReal(0,0,0), aaWorld(0,0,0), gravity(0,0,0)
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
    mpu.setZAccelOffset(1666);

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, &dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
    if(mpuInterrupt)
    {
        //reset interrupt
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else{
            while (!(mpuIntStatus & 0x02)){
                mpuIntStatus = mpu.getIntStatus(); // Wait until new data is available. (Nothing else to do)
            }
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            //Let's empty this damn fifo
            while(fifoCount >= packetSize){
                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
            }

            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);

            // offset quaternion
            q = qOffset.getProduct(q);

            //get EulerAngles phi theta psi
            mpu.dmpGetEuler(euler, &q);
            float phi = euler[2];
            euler[2] = -euler[0]; // Psi
            euler[0] = -phi; // Phi
            euler[1] = -euler[1]; // Theta

            //get Gyro rates
            mpu.dmpGetGyro(gyro, fifoBuffer);

            // get Gravity unity vector in body axis
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
    // The earth axis in Z pointing up and Y pointing left according to the IMU/DMP. But our system of axis is Z pointing down and Y pointing right.
    quat[0] = -q.x;
    quat[1] = q.w;
    quat[2] = q.z;
    quat[3] = -q.y;
}

void IMU::getEulerAngles(float* eulerAngles)
{
    Matrix.Copy((float*)euler,3,1,eulerAngles);
}

void IMU::getGyro(float* gyroRates)
{
    //Rotation rates conversion in rad/s
    gyroRates[0] = (float)gyro[0]*3.14f/130;
    gyroRates[1] = (float)gyro[1]*3.14f/130;
    gyroRates[2] = (float)gyro[2]*3.14f/130;
}

void IMU::getRelativeAcceleration(float* relativeAccel)
{
    //Acceleration conversion in m.s-2
    relativeAccel[0] = (float)aa.x*9.81f/8192;
    relativeAccel[1] = (float)aa.y*9.81f/8192;
    relativeAccel[2] = (float)aa.z*9.81f/8192;
}

void IMU::getLinearAcceleration(float *acceleration)
{
    //Acceleration conversion in m.s-2
    acceleration[0] = (float)aaReal.x*9.81f/8192;
    acceleration[1] = (float)aaReal.y*9.81f/8192;
    acceleration[2] = (float)aaReal.z*9.81f/8192;
}

void IMU::getEarthAcceleration(float *earthAccel)
{
    //Acceleration conversion in m.s-2
    // We just change z = -z and y = -y to have z pointing down and y pointing right
    earthAccel[0] = (float)aaWorld.x*9.81f/8192;
    earthAccel[1] = -(float)aaWorld.y*9.81f/8192;
    earthAccel[2] = -(float)aaWorld.z*9.81f/8192;
}

void IMU::resetGyroPath()
{
    qOffset = q.getConjugate();
}
