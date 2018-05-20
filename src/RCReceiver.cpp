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

    // Reading every channels values
    for(int i=0;i<6;i++)
    {
        run();
    }
}

void RCReceiver::run()
{
    pulseLength[pinRead] = pulseIn(pin[pinRead],HIGH,TIMEOUT); //pulse length reading (one channel at a time)
    pinRead = (pinRead+1)%4; // Incrementation of pinRead for next pulse read (Autopilot and aerobatics not checked)
}

float RCReceiver::getYawCommand()
{
    float yaw;
    yaw = YAWMIN + (YAWMAX-YAWMIN) * ((float)pulseLength[0]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return yaw;
}

float RCReceiver::getPitchCommand()
{
    float pitch;
    pitch = PITCHMIN + (PITCHMAX-PITCHMIN) * ((float)pulseLength[1]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return pitch;
}

float RCReceiver::getRollCommand()
{
    float roll;
    roll = ROLLMIN + (ROLLMAX-ROLLMIN) * ((float)pulseLength[2]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return roll;
}

float RCReceiver::getThrottleCommand()
{
    float throttle;
    throttle = THROTTLEMIN + (THROTTLEMAX-THROTTLEMIN) * ((float)pulseLength[3]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return throttle;
}

float RCReceiver::getChannel5()
{
    float channel5;
    channel5 = 100 * ((float)pulseLength[5]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    channel5 = max(channel5,0);
    channel5 = min(channel5,100);
    return channel5;
}

float RCReceiver::getAutopilotEnabled()
{
    if(pulseLength[5] > THRESHOLD)
        return true;
    else
        return false;
}

float RCReceiver::getChannel6()
{
    float channel6;
    channel6 = 100 * ((float)pulseLength[4]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    channel6 = max(channel6,0);
    channel6 = min(channel6,100);
    return channel6;
}

float RCReceiver::getAerobaticsEnabled()
{
    if(pulseLength[4] > THRESHOLD)
        return true;
    else
        return false;
}

bool RCReceiver::throttleDown()
{
    if(((float)pulseLength[3]-PULSEMIN) < (PULSEMAX-PULSEMIN)/10)
        return true;
    else return false;
}

bool RCReceiver::throttleUp()
{

    if(((float)pulseLength[3]-PULSEMIN) > (PULSEMAX-PULSEMIN)*9/10)
        return true;
    else return false;
}

bool RCReceiver::signalReceived()
{
    for(int i = 0; i<4;i++)
    {
        if(pulseLength[i] == 0)
            return false;
    }
    return true;
}
