#include "AttitudeManager.h"

AttitudeManager::AttitudeManager()
{
    // Time init
    m_lastTime = millis();
    m_period = 50; // milliseconds

    // Gain init
    float KPosition[4][4] = {0};
    float KSpeed[4][4] = {0};
    float KIntegral[4][4] = {0};
    setKPosition((float*) KPosition);
    setKSpeed((float*) KSpeed);
    setKIntegral((float*) KIntegral);

    // Reference & integral init
    float reference[4] = {0};
    setReference((float*) reference);
    resetIntegral();
    disableIntegral();

    // Motors init
    m_motor1.setPin(4);
    m_motor2.setPin(5);
    m_motor3.setPin(6);
    m_motor4.setPin(7);
}

// Public methods

void AttitudeManager::run()
// Default running
{
    // Update integral if the integral action is enabled
    if(integralEnabled()){
        updateIntegral();
    }

    float command[4] = {0};
    getCommand((float*) command);
    sendMotorOutput((float*) command);
}

bool AttitudeManager::periodElapsed()
// Returns true if the period of the AttitudeManager is elapsed
{
    if((unsigned)millis()-m_lastTime > m_period)
    {
        m_lastTime = millis();
        return true;
    }
    else
        return false;
}

void AttitudeManager::setKPosition(float* KPosition)
// Proportional gains
{
    Matrix.Copy(KPosition,4,4,(float*)m_KPosition);
}

void AttitudeManager::setKSpeed(float* KSpeed)
// Derivative gains
{
    Matrix.Copy(KSpeed,4,4,(float*)m_KSpeed);
}

void AttitudeManager::setKIntegral(float* KIntegral)
// Integral gains
{
    Matrix.Copy(KIntegral,4,4,(float*)m_KIntegral);
}

void AttitudeManager::setReference(float* reference)
// Set initial reference for control
{
    Matrix.Copy(reference,4,1,(float*)m_reference);
}

void AttitudeManager::getReference(float* reference)
{
    Matrix.Copy((float*)m_reference,4,1,(float*)reference);
}

void AttitudeManager::setPeriod(long period)
// Set period of run
{
    m_period = period;
}

// Private methods

void AttitudeManager::getCommand(float* command)
// Calculate motor commands (speed^2 with speed in tr/s)
{
    // State, Derivative, Integral and reference
    float state[4] = {0};
    float stateDerivative[4] = {0};
    float stateIntegral[4] = {0};
    float reference[4] = {0};
    float error[4] = {0};

    sensors->getAttitudeState((float*) state);
    sensors->getAttitudeDerivative((float*) stateDerivative);
    getIntegral((float*) stateIntegral);
    getReference((float*) reference);
    Util::compareAttitudes((float*)reference,(float*)state,(float*)error);

    //Following calculation :
    // Command = Kp*Error + Kv*stateDerivative + Ki*stateIntegral

    // proportional = KPosition*error
    float proportional[4] = {0};
    Matrix.Multiply((float*)m_KPosition,(float*)error,4,4,1,(float*)proportional);

    // derivative = KSpeed*stateDerivative
    float derivative[4] = {0};
    Matrix.Multiply((float*)m_KSpeed,(float*)stateDerivative,4,4,1,(float*)derivative);

    // sumPropDeriv = proportional + derivative;
    float sumPropDeriv[4] = {0};
    Matrix.Add((float*)proportional,(float*)derivative,4,1,(float*)sumPropDeriv);

    // integral = KIntegral*integral
    float integral[4] = {0};
    Matrix.Multiply((float*)m_KIntegral,(float*)stateIntegral,4,4,1,(float*)integral);

    // Command = proportional, integral, derivative
    Matrix.Add((float*)sumPropDeriv,(float*)integral,4,1,(float*)command);
}

void AttitudeManager::sendMotorOutput(float* command)
// Send to motors
{
    m_motor1.setCommand(command[0]);
    m_motor2.setCommand(command[1]);
    m_motor3.setCommand(command[2]);
    m_motor4.setCommand(command[3]);
}

void AttitudeManager::enableIntegral()
// Enable integral action (to enable close to final position)
{
    // Only if the integral was disabled
    if(!integralEnabled()){
        m_integralEnabled = true;
        resetIntegral();
    }
}

void AttitudeManager::disableIntegral()
// Disable integral action (to disable far from final position)
{
    m_integralEnabled = false;
}

bool AttitudeManager::integralEnabled()
{
    return m_integralEnabled;
}

void AttitudeManager::resetIntegral()
// Reset integral calculation. To use when suddenly changing reference.
{
    float integral[4] = {0};
    Matrix.Copy(integral,4,1,m_integral);
    m_integralLastTime = millis();
}

void AttitudeManager::updateIntegral()
// Update integral values.
{
    long dt = millis()-m_integralLastTime;
    m_integralLastTime = millis();

    float state[4] = {0}; sensors->getAttitudeState((float*)state);
    float reference[4] = {0}; getReference(reference);
    float error[4] = {0}; // Difference between ref and state
    Util::compareAttitudes((float*)reference,(float*)state,(float*)error);

    m_integral[0] = m_integral[0] + dt*error[0]/1000;
    m_integral[1] = m_integral[1] + dt*error[1]/1000;
    m_integral[2] = m_integral[2] + dt*error[2]/1000;
    m_integral[3] = m_integral[3] + dt*error[3]/1000;
}

void AttitudeManager::getIntegral(float* integral)
{
    Matrix.Copy(m_integral,4,1,integral);
}

bool AttitudeManager::isClose()
//TODO : Implementation of isClose()
{
    return false;
}
