#ifndef DEF_ATTITUDEMANAGER
#define DEF_ATTITUDEMANAGER

#include "Util.h"
#include <MatrixMath.h>
#include "Motor.h"
#include "Sensors.h"

class AttitudeManager
{
public:

    AttitudeManager();

    void run();
    bool periodElapsed();
    void setKPosition(float* KPosition);
    void setKSpeed(float* KSpeed);
    void setKIntegral(float* KIntegral);
    void setReference(float* reference);
    void setPeriod(long period);
    void getReference(float *reference);


private:

    long m_lastTime;
    long m_period;
    long m_integralLastTime;
    float m_reference[4];
    float m_integral[4];
    bool m_integralEnabled;
    Motor m_motor1;
    Motor m_motor2;
    Motor m_motor3;
    Motor m_motor4;
    float m_KPosition[4][4];
    float m_KSpeed[4][4];
    float m_KIntegral[4][4];

    void getCommand(float* command);
    void sendMotorOutput(float* command);
    void enableIntegral();
    void disableIntegral();
    bool integralEnabled();
    void resetIntegral();
    void updateIntegral();
    void getIntegral(float *integral);
    bool isClose();
};

extern AttitudeManager *attitudeManager;
#endif
