SOURCES := src/main.cpp src/Util.cpp src/AttitudeManager.cpp src/Motor.cpp src/Sensors.cpp src/IMU.cpp src/Master.cpp src/RCReceiver.cpp
LIBRARIES := MatrixMath Servo I2Cdev Wire MPU6050
BOARD := uno
include arduino.mk

