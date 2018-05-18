#include "MPU6050.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"


uint8_t dmpInitialize();

bool dmpPacketAvailable();

// uint8_t dmpSetFIFORate(uint8_t fifoRate);
// uint8_t dmpGetFIFORate();
// uint8_t dmpGetSampleStepSizeMS();
// uint8_t dmpGetSampleFrequency();
// int32_t dmpDecodeTemperature(int8_t tempReg);

//uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t dmpRunFIFORateProcesses();

// uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet) ;
uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet) ;
uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet) ;
uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet);
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet);
uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
// uint8_t dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet);
uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet);
uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet) ;
// uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);

// uint8_t dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) ;
// uint8_t dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t dmpGetGravity(long *data, const uint8_t* packet);
uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q) ;
// uint8_t dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t dmpGetEIS(long *data, const uint8_t* packet);

uint8_t dmpGetEuler(float *data, Quaternion *q);
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData) ;
uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
// uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t dmpInitFIFOParam();
// uint8_t dmpCloseFIFO();
// uint8_t dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t dmpDecodeQuantizedAccel();
// uint32_t dmpGetGyroSumOfSquare();
// uint32_t dmpGetAccelSumOfSquare();
// void dmpOverrideQuaternion(long *q);
uint16_t dmpGetFIFOPacketSize();

