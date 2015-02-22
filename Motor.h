#ifndef DEF_MOTOR_H
#define DEF_MOTOR_H

#include "Util.h"
#include <Servo.h>

class Motor
{
public :

    Motor();
    Motor(int pin);

    void setCommand(float command);
    void setPin(int pin);

private :
    Servo m_ESC;
};

#endif
