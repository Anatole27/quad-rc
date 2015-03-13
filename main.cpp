#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "MatrixMath.h"
#include "RCReceiver.h"

AttitudeManager* attitudeManager;
Sensors* sensors;
RCReceiver* receiver;

int main()
{
    init();
    int led = 13;

    // the setup routine runs once when you press reset:
    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("Debut de test"));

    //Test
    //Initialisation
    attitudeManager = new AttitudeManager();
    sensors = new Sensors();
    receiver = new RCReceiver();

    Serial.println(F("Start"));
    while(millis() < 100000){
        receiver->run();

        Serial.print(receiver->pulseLength[0]); Serial.print(F(","));
        Serial.print(receiver->pulseLength[1]); Serial.print(F(","));
        Serial.print(receiver->pulseLength[2]); Serial.print(F(","));
        Serial.print(receiver->pulseLength[3]); Serial.print(F(","));
        Serial.print(receiver->pulseLength[4]); Serial.print(F(","));
        Serial.println(receiver->pulseLength[5]);

        delay(0.01);
    }


    //Fin de test
    Serial.println(F("Test termined"));
    Serial.end();
    while(true){
        digitalWrite(led,HIGH);
        delay(100);
        digitalWrite(led,LOW);
        delay(100);
    }
}

