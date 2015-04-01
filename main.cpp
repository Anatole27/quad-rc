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

    float att[4] = {0};
    float deriv[4] = {0};
    Serial.println(F("Start"));
    while(true){
        master->run();
        attitudeManager->run();
        receiver->run();
        sensors->run();
        sensors->getAttitudeState(att);
        sensors->getAttitudeDerivative(deriv);
        Serial.print(millis()); Serial.print(F(","));
        Serial.print(att[1]); Serial.print(F(","));
        Serial.print(att[2]); Serial.print(F(","));
        Serial.print(att[3]); Serial.print(F(","));
        Serial.print(deriv[1]); Serial.print(F(","));
        Serial.print(deriv[2]); Serial.print(F(","));
        Serial.print(deriv[3]); Serial.print(F(","));
        Serial.print(attitudeManager->m_motor1.m_pulse); Serial.print(F(","));
        Serial.print(attitudeManager->m_motor2.m_pulse); Serial.print(F(","));
        Serial.print(attitudeManager->m_motor3.m_pulse); Serial.print(F(","));
        Serial.println(attitudeManager->m_motor4.m_pulse);
    }
}

