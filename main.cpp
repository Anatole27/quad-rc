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
    Serial.println(F("Start"));
    while(millis() < 10000){
        sensors->run();
        Serial.print(sensors->position[0]); Serial.print(F(","));
        Serial.print(sensors->position[1]); Serial.print(F(","));
        Serial.print(sensors->position[2]); Serial.print(F(","));
        Serial.print(sensors->speed[0]); Serial.print(F(","));
        Serial.print(sensors->speed[1]); Serial.print(F(","));
        Serial.print(sensors->speed[2]); Serial.print(F(","));
        Serial.print(sensors->acceleration[0]); Serial.print(F(","));
        Serial.print(sensors->acceleration[1]); Serial.print(F(","));
        Serial.print(sensors->acceleration[2]); Serial.print(F(","));
        Serial.print(sensors->earthAccel[0]); Serial.print(F(","));
        Serial.print(sensors->earthAccel[1]); Serial.print(F(","));
        Serial.print(sensors->earthAccel[2]); Serial.print(F(","));
        Serial.print(sensors->quaternion[0]); Serial.print(F(","));
        Serial.print(sensors->quaternion[1]); Serial.print(F(","));
        Serial.print(sensors->quaternion[2]); Serial.print(F(","));
        Serial.print(sensors->quaternion[3]); Serial.print(F(","));
        Serial.print(sensors->eulerAngles[0]); Serial.print(F(","));
        Serial.print(sensors->eulerAngles[1]); Serial.print(F(","));
        Serial.print(sensors->eulerAngles[2]); Serial.print(F(","));
        Serial.print(sensors->gyroRates[0]); Serial.print(F(","));
        Serial.print(sensors->gyroRates[1]); Serial.print(F(","));
        Serial.println(sensors->gyroRates[2]);
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

