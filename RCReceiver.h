#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include "Arduino.h"

#define PINYAW 4
#define PINPITCH 5
#define PINROLL 6
#define PINTHROTTLE 7
#define PINAUTOPILOT 8
#define PINAEROBATICS 9

#define YAWMIN -1.5f
#define YAWMAX 1.5f
#define PITCHMIN -0.79f // pi/4
#define PITCHMAX 0.79f
#define ROLLMIN -0.79f
#define ROLLMAX 0.79f
#define THROTTLEMIN -1.0f
#define THROTTLEMAX 1.0f
#define THRESHOLD 0.001 // autopilot and aerobatics threshold in seconds
#define PULSEMIN 0.0015
#define PULSEMAX 0.0035

#define TIMEOUT 0.04 // Maximum waiting time for a pulse in

class RCReceiver
{
public:
    RCReceiver();
    void run();
    float getYawCommand();
    float getPitchCommand();
    float getRollCommand();
    float getThrottleCommand();
    float getAutopilotEnabled();
    float getAerobaticsEnabled();

private:
    unsigned long pulseLength[6];
    int pin[6];
    int pinRead;
};

#endif // RCRECEIVERH
