#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include "Arduino.h"

#define PINYAW 12
#define PINPITCH 8
#define PINROLL 5
#define PINTHROTTLE 7
#define PINAUTOPILOT 3
#define PINAEROBATICS 4

#define YAWMIN -2.f
#define YAWMAX 2.f
#define PITCHMIN 0.4f // pi/4
#define PITCHMAX -0.4f
#define ROLLMIN -0.4f
#define ROLLMAX 0.4f
#define THROTTLEMIN 1.0f
#define THROTTLEMAX -1.0f
#define THRESHOLD 950 // autopilot and aerobatics threshold in micro seconds
#define PULSEMIN 870 // minimum pulse length in us
#define PULSEMAX 1730 // minimum pulse length in us
#define MIDPULSE 1300 // medium pulse length
#define DELTAPULSE 430 // midpule + deltapulse = maxpulse

#define TIMEOUT 20000 // Maximum waiting time for a pulse in micro sec

class RCReceiver
{
public:
    RCReceiver();
    void run(); // Read RX signals
    float getYawCommand(); // Translate pulse into yaw command
    float getPitchCommand(); // Translate pulse into pitch command
    float getRollCommand(); // Translate pulse into roll command
    float getThrottleCommand(); // Translate pulse into throttle command
    float getChannel5(); // Gives channel5 signal 0-100%
    float getChannel6(); // Gives channel6 signal 0-100%
    float getAutopilotEnabled(); // Translate pulse into autopilot enabling
    float getAerobaticsEnabled(); // Aerobatics enabling (attitude rate control)
    bool throttleUp(); // true when throttle is between 90% and 100%
    bool throttleDown(); // true when throttle is between 0% and 10%
    bool signalReceived();

//private:
    unsigned long pulseLength[6]; // Stores every channels signal length
    int pin[6]; // Stores pins numbers for channels 1 to 6
    int pinRead; // Pin read when run() is called
};

extern RCReceiver *receiver;

#endif // RCRECEIVERH
