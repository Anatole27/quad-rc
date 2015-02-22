#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "MatrixMath.h"

AttitudeManager* attitudeManager;
Sensors* sensors;

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

    float attitudeState[4] = {0};
    float attitudeDerivative[4] = {0};

    while(millis() < 1000){
        sensors->run();
        sensors->getAttitudeState((float*)attitudeState);
        sensors->getAttitudeDerivative((float*)attitudeDerivative);
        Matrix.Print(attitudeState,4,1,"State");
        Matrix.Print(attitudeDerivative,4,1,"Derivative");
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

