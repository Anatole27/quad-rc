#ifndef DEF_MOTOR_H
#define DEF_MOTOR_H

#include "Util.h"
#include "Arduino.h"
#include <Servo.h>

#define MAXPULSE 2000 //Max pulse length (us)
#define MINPULSE 1000 //Min pulse length (us)
#define NOMINALSPEED 5600l // Nominal speed when hovering (tr/min)
#define MAXSPEED 11000 // tr/min
#define MINSPEED 0 // tr/min

class Motor
{
public :

    Motor();
    Motor(int pin);

    void setCommand(float command);
    void setPin(int pin);
    void setMaxPulse();
    void setMinPulse(); // set Max/Min pulse to calibrate ESC

private :
    Servo m_ESC;
};

#endif
