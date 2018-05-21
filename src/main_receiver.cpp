#include <Arduino.h>
#include "MatrixMath.h"
#include "RCReceiver.h"

RCReceiver* receiver;

long meanPeriod = 0;


int main()
{
    init();
    int led = 13;
    unsigned long time;

    // the setup routine runs once when you press reset:
    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial.println(F("Debut de test"));

    //Test
    //Initialisation
    receiver = new RCReceiver();

    Serial.println(F("Start"));
    while(true){
        time = micros();

        // Receiver
        receiver->run();

        // Debug outputs
        Serial.print(micros() - time);
        Serial.print(" ");
        Serial.print(receiver->pulseLength[2]); // Roll
        Serial.print(" ");
        Serial.print(receiver->pulseLength[1]); // Pitch
        Serial.print(" ");
        Serial.print(receiver->pulseLength[0]); // Yaw
        Serial.print(" ");
        Serial.print(receiver->pulseLength[3]); // Throttle
        Serial.println(" ");
    }
}
