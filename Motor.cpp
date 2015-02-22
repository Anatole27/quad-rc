#include "Motor.h"

Motor::Motor()
{
    m_ESC.attach(4);
}

Motor::Motor(int pin)
{
    m_ESC.attach(pin);
}

void Motor::setCommand(float command)
//TODO : implement conversion. Reminder : command = n^2 witch n = tr/s.
{
    m_ESC.writeMicroseconds(700);
}

void Motor::setPin(int pin)
{
    m_ESC.detach();
    m_ESC.attach(pin);
}
