#ifndef DEF_MOTOR_H
#define DEF_MOTOR_H

#include "Util.h"
#include "Arduino.h"
#include <Servo.h>

#define MAXPULSE 2000 //Max pulse length (us)
#define MINPULSE 1000 //Min pulse length (us)
#define DEFAULTNOMINALTHRUST 250.0 // Nominal speed when hovering (tr/min)
#define MINTHRUST 0.0 // gramms
#define MAXTHRUST 500.0 // gramms
#define X2_COEF 820.0
#define X_COEF 157.0

class Motor
{
public :

    Motor();
    Motor(int pin);

    void setCommand(float command);
    void setPin(int pin);
    int getPin();
    void setMaxPulse();
    void setMinPulse(); // set Max/Min pulse to calibrate ESC
    void setPulse(int pulse);
    void setNominalSpeed(long nominalSpeed);
    long getNominalSpeed();

//private :
    Servo m_ESC;
    int m_pin;
    long m_nominalThrust;
    unsigned long m_pulse;
};

#endif
