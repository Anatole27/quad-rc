#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "MatrixMath.h"
#include "RCReceiver.h"
#include "Master.h"

#define SAMPLE_TIME 25 //ms

void setGains(AttitudeManager *attitudeManager);

AttitudeManager* attitudeManager;
Sensors* sensors;
RCReceiver* receiver;
Master* master;

int main()
{
    init();
    int led = 13;
    long time = 0;

    // the setup routine runs once when you press reset:
    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial.println(F("Debut de test"));

    //Test
    //Initialisation
    attitudeManager = new AttitudeManager();
    sensors = new Sensors();
    receiver = new RCReceiver();
    master = new Master();

    //Initialization
    master->init();

    float state[6] = {0};
    float ref[4] = {0};
    float Kposition[4][4];
    float Kspeed[4][4];
    Serial.println(F("Start"));
    while(true){
        if(millis()-time > SAMPLE_TIME){ // 40 Hz
            time = millis();
            master->run();
            attitudeManager->run();
            receiver->run();
            sensors->run();
            setGains(attitudeManager);

            sensors->getEulerAngles(state);
            sensors->getGyroRates(state + 3);
            attitudeManager->getReference(ref);

            Serial.print(millis()); Serial.print(F(","));

            for(int i = 0;i<6;i++){
                Serial.print(state[i],4); Serial.print(F(","));}
            for(int i = 1;i<4;i++)
            {Serial.print(ref[i],4); Serial.print(F(","));}

            Serial.print(attitudeManager->m_motor1.m_pulse); Serial.print(F(","));
            Serial.print(attitudeManager->m_motor2.m_pulse); Serial.print(F(","));
            Serial.print(attitudeManager->m_motor3.m_pulse); Serial.print(F(","));
            Serial.print(attitudeManager->m_motor4.m_pulse); Serial.print(F(","));

            attitudeManager->getKPosition((float*)Kposition);
            attitudeManager->getKSpeed((float*)Kspeed);
            Serial.print(Kposition[0][3]); Serial.print(F(","));
            Serial.println(-Kspeed[0][3]);
        }
        delay(1);
    }
}

void setGains(AttitudeManager* attitudeManager)
{
    String number;   // Gain
    String fb; // Gain type
    number = NULL;
    fb = NULL;
    float K[4] = {0};

    if(Serial.available() > 0){
        // read the incoming byte:
        number = Serial.readStringUntil('K');
        if(number != NULL){
            fb = Serial.readString();

            if(fb.compareTo("p") == 0){
                Serial.println(F("Change"));
                K[3] = (float)number.toInt()*51100.f;
                attitudeManager->setKPosition(K);}

            if(fb.compareTo("v") == 0){
                Serial.println(F("Change"));
                K[3] = (float)number.toInt()*23280.f;
                attitudeManager->setKSpeed(K);}

            number = NULL;
        }
    }
}
