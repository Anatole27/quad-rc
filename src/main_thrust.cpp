#include <Arduino.h>
#include "Sensors.h"
#include "AttitudeManager.h"
#include "RCReceiver.h"
#include "Master.h"

#define SAMPLE_TIME 25 //ms  --> 40 Hz
#define MOTORPIN3 10

void getCmd();
bool waitInput();
Motor motor3;

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

    //Initialization
    motor3.setPin(MOTORPIN3);
    Serial.println(F("ESC Calibration"));
    Serial.println(F("Throttle Up"));
    motor3.setMaxPulse(); // Set maximum length pulse

    // Wait for any input from serial
    while(!waitInput()){
        delay(10);
    }

    Serial.println(F("Throttle Down"));
    motor3.setMinPulse(); // Set minimum length pulse
    delay(3000); // wait for the ESC to copy correctly

    Serial.println(F("Start"));
    while(true){
        if(millis()-time > SAMPLE_TIME){

            // Main loop
            getCmd(); // Update cmd

        }
        delay(1);
    }
}

void getCmd()
{
    String number;   // Gain
    number = NULL;

    if(Serial.available() > 0){
        // read the incoming byte:
        number = Serial.readString();
        if(number != NULL){
            if(number.toInt() >= 1000 && number.toInt() <= 2000){
            Serial.print("New cmd : ");
            Serial.println(number);
            motor3.setPulse(number.toInt());
            }
            else{
                Serial.println("Emergency cut-off");
                motor3.setPulse(1000);
            }
        }
    }
}

bool waitInput()
{
    String input;   // Gain
    input = NULL;

    if(Serial.available() > 0){
        // read the incoming byte:
        input = Serial.readString();
        if(input != NULL){
            return true;
        }
    }
    return false;
}
