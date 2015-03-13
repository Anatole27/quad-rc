#include "RCReceiver.h"

RCReceiver::RCReceiver()
{
    for(int i = 0;i<6;i++)
    {
        pulseLength[i] = 0;
        pinRead = 0;
    }

    // Setting pin numbering
    pin[0] = PINYAW;
    pin[1] = PINPITCH;
    pin[2] = PINROLL;
    pin[3] = PINTHROTTLE;
    pin[4] = PINAUTOPILOT;
    pin[5] = PINAEROBATICS;

    // pin INPUT enabling
    for(int i = 0;i<6;i++)
    {
        pinMode(pin[i], INPUT);
    }
}

void RCReceiver::run()
{
    pulseLength[pinRead] = pulseIn(pin[pinRead],HIGH,TIMEOUT); //pulse length reading (one channel at a time)
    pinRead = (pinRead+1)%6; // Incrementation of pinRead for next pulse read
}

float RCReceiver::getYawCommand()
{
    float yaw;
    yaw = YAWMIN + (YAWMAX-YAWMIN)*(float)pulseLength[0]/(PULSEMAX-PULSEMIN);
    return yaw;
}

float RCReceiver::getPitchCommand()
{
    float pitch;
    pitch = PITCHMIN + (PITCHMAX-PITCHMIN)*(float)pulseLength[1]/(PULSEMAX-PULSEMIN);
    return pitch;
}

float RCReceiver::getRollCommand()
{
    float roll;
    roll = ROLLMIN + (ROLLMAX-ROLLMIN)*(float)pulseLength[2]/(PULSEMAX-PULSEMIN);
    return roll;
}

float RCReceiver::getThrottleCommand()
{
    float throttle;
    throttle = THROTTLEMIN + (THROTTLEMAX-THROTTLEMIN)*(float)pulseLength[3]/(PULSEMAX-PULSEMIN);
    return throttle;
}

float RCReceiver::getAutopilotEnabled()
{
    if(pulseLength[4] > THRESHOLD)
        return true;
    else
        return false;
}

float RCReceiver::getAerobaticsEnabled()
{
    if(pulseLength[5] > THRESHOLD)
        return true;
    else
        return false;
}

