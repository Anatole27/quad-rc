#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "MatrixMath.h"
#include "RCReceiver.h"
#include "Master.h"

AttitudeManager* attitudeManager;
Sensors* sensors;
RCReceiver* receiver;
Master* master;

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
    // On passe ici
    sensors = new Sensors();
    receiver = new RCReceiver();
    master = new Master();

    //Initialization
    //master->init();

    long t;

    Serial.println(F("Start"));
    while(true){
        Serial.print(millis()); Serial.print(F(","));

        t = millis();
        master->run();
        Serial.print(millis()-t); Serial.print(F(","));

        t = millis();
        attitudeManager->run();
        Serial.print(millis()-t); Serial.print(F(","));

        t = millis();
        receiver->run();
        Serial.print(millis()-t); Serial.print(F(","));

        t = millis();
        sensors->run();
        Serial.println(millis()-t);

        // Total time 25ms
    }
}


