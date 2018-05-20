#include "AttitudeManager.h"

AttitudeManager::AttitudeManager()
{
    // Time init
    m_lastTime = millis();
    m_period = 50; // milliseconds

    // Gain init
    //    float KP[4] = {3546682.f,   5110000.f,   5110000.f,    51100000.f};
    //    float KS[4] = {0.f,    2328000.f,    2328000.f,   4656000.f};
    float KP[4] = {125.f,   0.f,   0.f,    0.f};
    float KS[4] = {0.f,    0.f,    0.f,   0.f};
    float KI[4] = {0};

    setKPosition((float*) KP);
    setKSpeed((float*) KS);
    setKIntegral((float*) KI);

    // Reference & integral init
    float reference[4] = {0};
    setReference((float*) reference);
    resetIntegral();
    disableIntegral();

    // Motors init
    m_motor1.setPin(MOTORPIN1);
    m_motor2.setPin(MOTORPIN2);
    m_motor3.setPin(MOTORPIN3);
    m_motor4.setPin(MOTORPIN4);

    // Boolean init
    managerEnabled = false;
}

// Public methods

void AttitudeManager::run()
// Default running
{
    if(managerEnabled)
    {
        // Update integral if the integral action is enabled
        if(integralEnabled()){
            updateIntegral();
        }

        float command[4] = {0};
        getCommand((float*) command);
        sendMotorOutput((float*) command);
    }
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

void AttitudeManager::setKPosition(float* KP)
// Proportional gains
{
    float KPosition[4][4] = {
        {-1,   -1,   1,    1},
        {-1,   -1,   -1,   -1},
        {-1,   1,    -1,    1},
        {-1,   1,    1,   -1}};

    for(int i = 0; i< 4; i++)
        for(int j = 0; j<4; j++)
            KPosition[i][j] = KPosition[i][j]*KP[j];

    Matrix.Copy((float*)KPosition,4,4,(float*)m_KPosition);
}

void AttitudeManager::setKSpeed(float* KS)
// Derivative gains
{
    float KSpeed[4][4] = {
        {-1,   1,   -1,   -1},
        {-1,   1,   1,   1},
        {-1,   -1,  1,   -1},
        {-1,   -1,  -1,   1}};

    for(int i = 0; i< 4; i++)
        for(int j = 0; j<4; j++)
            KSpeed[i][j] = KSpeed[i][j]*KS[j];

    Matrix.Copy((float*)KSpeed,4,4,(float*)m_KSpeed);
}

void AttitudeManager::setKIntegral(float* KI)
// Integral gains
{
    float KIntegral[4][4] = {
        {-1,   -1,   1,    1},
        {-1,   -1,   -1,   -1},
        {-1,   1,    -1,    1},
        {-1,   1,    1,   -1}};

    for(int i = 0; i< 4; i++)
        for(int j = 0; j<4; j++)
            KIntegral[i][j] = KIntegral[i][j]*KI[j];

    Matrix.Copy((float*)KIntegral,4,4,(float*)m_KIntegral);
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
// Calculate motor commands
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

//    for(int i=0; i<4; i++){
//        Serial.print(" ");
//        Serial.print(error[i]);
//    }
//    Serial.print(" ");

    //Following calculations :
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

void AttitudeManager::setMaxPulse()
{
    m_motor1.setMaxPulse();
    m_motor2.setMaxPulse();
    m_motor3.setMaxPulse();
    m_motor4.setMaxPulse();
}

void AttitudeManager::testMotor(int i)
{
    Motor* motor;
    switch(i){
    case 0:
        motor = &m_motor1;
        break;
    case 1:
        motor = &m_motor2;
        break;
    case 2:
        motor = &m_motor3;
        break;
    case 3:
        motor = &m_motor4;
        break;
    default:
        return;
    }

    motor->setMaxPulse();
    delay(200);
    motor->setMinPulse();
    delay(800);
}

void AttitudeManager::setMinPulse()
{
    m_motor1.setMinPulse();
    m_motor2.setMinPulse();
    m_motor3.setMinPulse();
    m_motor4.setMinPulse();
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

void AttitudeManager::enable()
{
    managerEnabled = true;
}

void AttitudeManager::disable()
{
    managerEnabled = false;
}

void AttitudeManager::setNominalSpeed(long nominalSpeed)
{
    m_motor1.setNominalSpeed(nominalSpeed);
    m_motor2.setNominalSpeed(nominalSpeed);
    m_motor3.setNominalSpeed(nominalSpeed);
    m_motor4.setNominalSpeed(nominalSpeed);
}

void AttitudeManager::getKPosition(float *Kp)
{
    for(int i = 0; i<4; i++){
        Kp[i] = abs(m_KPosition[0][i]);
    }
}

void AttitudeManager::getKSpeed(float *Kv)
{
    for(int i = 0; i<4; i++){
        Kv[i] = abs(m_KSpeed[0][i]);
    }
}
