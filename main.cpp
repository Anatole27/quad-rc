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
    sensors = new Sensors();
    receiver = new RCReceiver();
    master = new Master();

    //Initialization
    master->init();

    while(true){
        master->run();
        attitudeManager->run();
        receiver->run();
        sensors->run();
        delay(10);
        // Serial output from attitudeManager SendOutPut and Motor.sendOutput
    }
}

