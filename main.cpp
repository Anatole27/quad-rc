#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include <MatrixMath.h>

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

    //Attitude Manager
    /*void run();*/
    //setReference
    float ref[4] = {21,2,4,2};
    attitudeManager->setReference((float*)ref);
    //getReference
    Serial.println(F(".getRef()"));
    attitudeManager->getReference((float*)ref);
    Matrix.Print(ref,4,1,"getRef");
    //periodElapsed()
    //    while(millis() < 1000)
    //        if(attitudeManager->periodElapsed())
    //            Serial.println(millis());
    //    attitudeManager->setPeriod(100);
    //    while(millis() < 2000)
    //        if(attitudeManager->periodElapsed())
    //            Serial.println(millis());
    //matrix K
    float Kpos[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    float Kspeed[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
    float Kint[16] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
    attitudeManager->setKIntegral((float*)Kint);
    attitudeManager->setKPosition((float*)Kpos);
    attitudeManager->setKSpeed((float*)Kspeed);
    //    Matrix.Print((float*)attitudeManager->m_KIntegral,4,4,"Integral");
    //    Matrix.Print((float*)attitudeManager->m_KSpeed,4,4,"Speed");
    //    Matrix.Print((float*)attitudeManager->m_KPosition,4,4,"Position");
    //Run
    while(millis() < 1000){
        if(attitudeManager->periodElapsed())
            attitudeManager->run();
    }
    //Integral





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

