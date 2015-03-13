#ifndef DEF_ATTITUDEMANAGER_H
#define DEF_ATTITUDEMANAGER_H

#include "Util.h"
#include <MatrixMath.h>
#include "Motor.h"
#include "Sensors.h"

#define MOTORPIN1 6
#define MOTORPIN2 9
#define MOTORPIN3 10
#define MOTORPIN4 11

class AttitudeManager
{
public:

    AttitudeManager();

    void run(); // update command/control
    bool periodElapsed(); // True if period since last execution elapsed. Reinitialize automatically
    void setKPosition(float* KPosition); // Position gains
    void setKSpeed(float* KSpeed); // Speed gains
    void setKIntegral(float* KIntegral); // Integral gains
    void setReference(float* reference); // Reference : z,phi,theta,psi (roll,pitch,yaw)
    void setPeriod(long period); // Period of run execution
    void getReference(float *reference);
    void setMaxPulse();
    void setMinPulse();


//private:

    long m_lastTime;
    long m_period;
    long m_integralLastTime;
    float m_reference[4];
    float m_integral[4];
    bool m_integralEnabled;
    Motor m_motor1; // Fore-right
    Motor m_motor2; // Aft-Right
    Motor m_motor3; // Aft-Left
    Motor m_motor4; // Fore-left

    // PID gains
    float m_KPosition[4][4];
    float m_KSpeed[4][4];
    float m_KIntegral[4][4];

    void getCommand(float* command); // Get motor command  = n^2-n0^2 in tr/min
    void sendMotorOutput(float* command); // Send motor PWM command on arduino pin
    void enableIntegral(); // integral action enabling
    void disableIntegral(); // desabling
    bool integralEnabled(); // test if integral action enabled or disabled
    void resetIntegral(); // reset integral (for waypoint changing)
    void updateIntegral(); // update integral (not automatic)
    void getIntegral(float *integral); // gives integral calculation
    bool isClose(); // True when reference is close to state
};

extern AttitudeManager *attitudeManager;
#endif
