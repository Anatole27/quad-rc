#include "RCReceiver.h"
#include <EnableInterrupt.h>

// TODO, si noInterrupts est declenche pendant un pulse, on risque de ne pas voir le front descendant

// Variables used for callbacks
uint32_t rcStart[6];
volatile uint16_t rcShared[6];

// Interrupt callbacks
void callbackChan(uint8_t axisIdx, uint8_t pinNb){
    if (digitalRead(pinNb) == HIGH) {
      rcStart[axisIdx] = micros();
    } else {
      rcShared[axisIdx] = (uint16_t)(micros() - rcStart[axisIdx]);
    }
}

void callbackYaw(){ callbackChan(YAWIDX, PINYAW); }
void callbackPitch(){ callbackChan(PITCHIDX, PINPITCH); }
void callbackRoll(){ callbackChan(ROLLIDX, PINROLL); }
void callbackThrottle(){ callbackChan(THROTTLEIDX, PINTHROTTLE); }
void callbackAutopilot(){ callbackChan(AUTOPILOTIDX, PINAUTOPILOT); }
void callbackAerobatics(){ callbackChan(AEROBATICSIDX, PINAEROBATICS); }

RCReceiver::RCReceiver()
{
    for(int i = 0;i<6;i++)
    {
        pulseLength[i] = 0;
        pinRead = 0;
    }

    // Setting pin numbering
    pin[YAWIDX] = PINYAW;
    pin[PITCHIDX] = PINPITCH;
    pin[ROLLIDX] = PINROLL;
    pin[THROTTLEIDX] = PINTHROTTLE;
    pin[AUTOPILOTIDX] = PINAUTOPILOT;
    pin[AEROBATICSIDX] = PINAEROBATICS;

    // pin INPUT enabling
    for(int i = 0;i<6;i++)
    {
        pinMode(pin[i], INPUT);
    }

    // Enable interrupts for each channel
    enableInterrupt(PINYAW, callbackYaw, CHANGE);
    enableInterrupt(PINPITCH, callbackPitch, CHANGE);
    enableInterrupt(PINROLL, callbackRoll, CHANGE);
    enableInterrupt(PINTHROTTLE, callbackThrottle, CHANGE);
    enableInterrupt(PINAUTOPILOT, callbackAutopilot, CHANGE);
    enableInterrupt(PINAEROBATICS, callbackAerobatics, CHANGE);
}

//void RCReceiver::run()
//{
//    pulseLength[pinRead] = pulseIn(pin[pinRead],HIGH,TIMEOUT); //pulse length reading (one channel at a time)
//    pinRead = (pinRead+1)%4; // Incrementation of pinRead for next pulse read (Autopilot and aerobatics not checked)
//}

float RCReceiver::getYawCommand()
{
    float yaw;
    yaw = YAWMIN + (YAWMAX-YAWMIN) * ((float)pulseLength[YAWIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return yaw;
}

float RCReceiver::getPitchCommand()
{
    float pitch;
    pitch = PITCHMIN + (PITCHMAX-PITCHMIN) * ((float)pulseLength[PITCHIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return pitch;
}

float RCReceiver::getRollCommand()
{
    float roll;
    roll = ROLLMIN + (ROLLMAX-ROLLMIN) * ((float)pulseLength[ROLLIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return roll;
}

float RCReceiver::getThrottleCommand()
{
    float throttle;
    throttle = THROTTLEMIN + (THROTTLEMAX-THROTTLEMIN) * ((float)pulseLength[THROTTLEIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    return throttle;
}

float RCReceiver::getChannel5()
{
    float channel5;
    channel5 = 100 * ((float)pulseLength[AUTOPILOTIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    channel5 = max(channel5,0);
    channel5 = min(channel5,100);
    return channel5;
}

float RCReceiver::getAutopilotEnabled()
{
    if(pulseLength[AUTOPILOTIDX] > THRESHOLD)
        return true;
    else
        return false;
}

float RCReceiver::getChannel6()
{
    float channel6;
    channel6 = 100 * ((float)pulseLength[AEROBATICSIDX]-PULSEMIN) / (PULSEMAX-PULSEMIN);
    channel6 = max(channel6,0);
    channel6 = min(channel6,100);
    return channel6;
}

float RCReceiver::getAerobaticsEnabled()
{
    if(pulseLength[AEROBATICSIDX] > THRESHOLD)
        return true;
    else
        return false;
}

bool RCReceiver::throttleDown()
{
    if(((float)pulseLength[THROTTLEIDX]-PULSEMIN) < (PULSEMAX-PULSEMIN)/10)
        return true;
    else return false;
}

bool RCReceiver::throttleUp()
{

    if(((float)pulseLength[THROTTLEIDX]-PULSEMIN) > (PULSEMAX-PULSEMIN)*9/10)
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

void RCReceiver::run() {
  noInterrupts();
  memcpy(pulseLength, (const void *)rcShared, sizeof(rcShared));
  interrupts();
}

