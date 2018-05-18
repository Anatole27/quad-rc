#include "Motor.h"

Motor::Motor() : m_pin(0),m_nominalSpeed(DEFAULTNOMINALSPEED),m_pulse(1000)
{
    m_ESC.attach(6);
}

Motor::Motor(int pin) : m_nominalSpeed(DEFAULTNOMINALSPEED),m_pulse(1000)
{
    m_pin = pin;
    m_ESC.attach(pin);
}

void Motor::setCommand(float command)
//TODO : implement conversion. Reminder : command = n^2 - n0^2 witch n = tr/min
{
    m_pulse = MINPULSE +
            (MAXPULSE-MINPULSE)*
            (sqrt(abs(command + m_nominalSpeed*m_nominalSpeed))-MINSPEED)
            /(MAXSPEED-MINSPEED);
    m_pulse = min(MAXPULSE,m_pulse);
    m_pulse = max(MINPULSE,m_pulse);
    m_ESC.writeMicroseconds(m_pulse);
}

void Motor::setMaxPulse()
{
    m_pulse = MAXPULSE;
    m_ESC.writeMicroseconds(MAXPULSE);
}

void Motor::setMinPulse()
{
    m_pulse = MINPULSE;
    m_ESC.writeMicroseconds(MINPULSE);
}

void Motor::setPin(int pin)
{
    m_pin = pin;
    m_ESC.detach();
    m_ESC.attach(pin);
}

int Motor::getPin()
{
    return m_pin;
}

void Motor::setNominalSpeed(long nominalSpeed)
{
    m_nominalSpeed = nominalSpeed;
}

long Motor::getNominalSpeed()
{
    return m_nominalSpeed;
}
