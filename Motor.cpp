#include "Motor.h"

Motor::Motor()
{
    m_ESC.attach(6);
}

Motor::Motor(int pin)
{
    m_ESC.attach(pin);
}

void Motor::setCommand(float command)
//TODO : implement conversion. Reminder : command = n^2 - n0^2 witch n = tr/min
{
    unsigned long pulse;
    pulse = (MAXPULSE-MINPULSE)*sqrt(command + NOMINALSPEED*NOMINALSPEED)/(MAXSPEED-MINSPEED);
    pulse = min(MAXPULSE,pulse);
    pulse = max(MINPULSE,pulse);
    m_ESC.writeMicroseconds(pulse);
}

void Motor::setMaxPulse()
{
    m_ESC.writeMicroseconds(MAXPULSE);
}

void Motor::setMinPulse()
{
    m_ESC.writeMicroseconds(MINPULSE);
}

void Motor::setPin(int pin)
{
    m_ESC.detach();
    m_ESC.attach(pin);
}
