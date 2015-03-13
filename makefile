SOURCES := main.cpp Util.cpp AttitudeManager.cpp Motor.cpp Sensors.cpp IMU.cpp Master.cpp RCReceiver.cpp
LIBRARIES := MatrixMath Servo I2Cdev Wire MPU6050
BOARD := uno
include arduino.mk

