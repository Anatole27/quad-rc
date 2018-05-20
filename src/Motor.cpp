#include "Motor.h"

Motor::Motor() : m_pin(0),m_nominalThrust(DEFAULTNOMINALTHRUST),m_pulse(1000)
{
    m_ESC.attach(6);
}

Motor::Motor(int pin) : m_nominalThrust(DEFAULTNOMINALTHRUST),m_pulse(1000)
{
    m_pin = pin;
    m_ESC.attach(pin);
}

void Motor::setCommand(float command)
// command = thrust - nominalThrust in gramms
{
    float thrust = command + m_nominalThrust; // gramms

    // Bounds
    thrust = max(MINTHRUST,min(MAXTHRUST, thrust));

    // Conversion from thrust to pulses
    m_pulse = MINPULSE*(1 + (-X_COEF + sqrt(X_COEF*X_COEF + 4*X2_COEF*thrust))/(2*X2_COEF));

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

void Motor::setPulse(int pulse)
{
    m_pulse = pulse;
    m_ESC.writeMicroseconds(pulse);
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
    m_nominalThrust = nominalSpeed;
}

long Motor::getNominalSpeed()
{
    return m_nominalThrust;
}
