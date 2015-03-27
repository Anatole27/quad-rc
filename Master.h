#ifndef MASTER_H
#define MASTER_H

#include "Arduino.h"
#include "AttitudeManager.h"
#include "RCReceiver.h"
#include "Sensors.h"

#define INITIAL_STATE 0
#define ESC_START 1
#define ESC_CALIBRATION 2
#define IMU_INIT 3
#define END_INIT 4
#define FLY_STATE 5
#define LANDING 6
#define SIGNAL_LOST 7

class Master
{
public:
    Master();
    void run();
    void init(); // Initialization procedure

private:
    int m_state;
    int m_initState;
};

extern Master* master;

#endif // MASTER_H
