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

    //Test
    //Initialisation
    attitudeManager = new AttitudeManager();
    // On passe ici
    sensors = new Sensors();
    receiver = new RCReceiver();
    master = new Master();

    //Initialization
    //master->init();
    float eulerAngles[3] = {0};

    while(true){
        sensors->run();
        sensors->getEulerAngles(eulerAngles);
        Serial.print(eulerAngles[0]);
        Serial.print(" ");
        Serial.print(eulerAngles[1]);
        Serial.print(" ");
        Serial.println(eulerAngles[2]);
        // Total time 25ms
    }
}


