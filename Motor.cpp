#include "Motor.h"

Motor::Motor() : m_pin(0),m_nominalSpeed(DEFAULTNOMINALSPEED)
{
    m_ESC.attach(6);
}

Motor::Motor(int pin) : m_nominalSpeed(DEFAULTNOMINALSPEED)
{
    m_pin = pin;
    m_ESC.attach(pin);
}

void Motor::setCommand(float command)
//TODO : implement conversion. Reminder : command = n^2 - n0^2 witch n = tr/min
{
    unsigned long pulse;
    pulse = MINPULSE +
            (MAXPULSE-MINPULSE)*
            (sqrt(abs(command + m_nominalSpeed*m_nominalSpeed))-MINSPEED)
            /(MAXSPEED-MINSPEED);
    pulse = min(MAXPULSE,pulse);
    pulse = max(MINPULSE,pulse);
    Serial.print(pulse);
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
