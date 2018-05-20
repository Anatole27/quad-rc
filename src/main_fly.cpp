#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "MatrixMath.h"
#include "RCReceiver.h"
#include "Master.h"

void setGains(AttitudeManager *attitudeManager);

AttitudeManager* attitudeManager;
Sensors* sensors;
RCReceiver* receiver;
Master* master;

long meanPeriod = 0;


int main()
{
    init();
    int led = 13;
    float state[6] = {0};
    float cmd[4] = {0};
    float ref[4] = {0};
    unsigned long time;

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

    Serial.println(F("Start"));
    while(true){
        time = millis();

        // Master
        master->run();

        // Attitude manager
        attitudeManager->run();

        // Receiver
        receiver->run();

        // Sensors
        sensors->run();

        // Other
        setGains(attitudeManager); // Update gains listening serial


        // Debug outputs
        sensors->getEulerAngles(state);
        sensors->getGyroRates(state + 3);
        attitudeManager->getReference(ref);
        attitudeManager->getCommand(cmd);

                    for(int i = 0;i<3;i++){
                        Serial.print(state[i],4); Serial.print(F(" "));}
                    for(int j = 0;j<4;j++){
                        Serial.print(cmd[j]/100.0,4); Serial.print(F(" "));}
                    for(int i = 1;i<4;i++)
                    {Serial.print(ref[i],4); Serial.print(F(" "));}
//                    Serial.println("");

                    Serial.print(attitudeManager->m_motor1.m_pulse); Serial.print(F(" "));
                    Serial.print(attitudeManager->m_motor2.m_pulse); Serial.print(F(" "));
                    Serial.print(attitudeManager->m_motor3.m_pulse); Serial.print(F(" "));
                    Serial.print(attitudeManager->m_motor4.m_pulse); Serial.print(F(" "));
                    Serial.println();

        //            attitudeManager->getKPosition((float*)Kposition);
        //            attitudeManager->getKSpeed((float*)Kspeed);
        //            Serial.print(Kposition[0][3]); Serial.print(F(","));
        //            Serial.println(-Kspeed[0][3]);
                    Serial.println(millis() - time);
    }
}

void setGains(AttitudeManager* attitudeManager)
{
    String number;   // Gain
    String fb; // Gain type
    String axis;
    number = NULL;
    fb = NULL;
    float K[4] = {0};

    if(Serial.available() > 0){
        // read the incoming byte:
        number = Serial.readStringUntil(' ');
        if(number != NULL){
            fb = Serial.readStringUntil(' ');
            axis = Serial.readString();

            if(fb.compareTo("Kp") == 0){
                attitudeManager->getKPosition(K);
                K[axis.toInt()] = (float)number.toInt();
                attitudeManager->setKPosition(K);
                Serial.print(F("New Kp["));
                Serial.print(axis);
                Serial.print(F("] = "));
                Serial.println(K[axis.toInt()]);

                Serial.print(F("Kp = "));
                for(int i = 0; i<4; i++){
                    Serial.print(K[i]);
                    Serial.print(" ");
                }
                Serial.println("");

            }

            if(fb.compareTo("Kv") == 0){
                attitudeManager->getKSpeed(K);
                K[axis.toInt()] = (float)number.toInt();
                attitudeManager->setKSpeed(K);
                Serial.print(F("New Kv["));
                Serial.print(axis);
                Serial.print(F("] = "));
                Serial.println(K[axis.toInt()]);
            }

            number = NULL;
        }
    }
}
