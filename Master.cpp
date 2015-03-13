#include "Master.h"

Master::Master()
{
    m_initState = INITIAL_STATE;
    m_state = FLY_STATE;
}

void Master::init()
{
    while(m_initState != END_INIT){

        // Stateflow
        switch(m_initState){

        //Initial state. Go to ESC start if throttle down, go to ESC Calibration if throttle up
        case INITIAL_STATE :
            receiver->run(); // Read receiver
            if(receiver->throttleDown()){
                Serial.println(F("Throttle Down"));
                m_initState = ESC_START;
            }
            else if(receiver->throttleUp()){
                Serial.println(F("Throttle Up"));
                m_initState = ESC_CALIBRATION;
            }
            break;

            //  ESC Calibration : Set all motor commands to max pulse then min pulse. Go to IMU initialization
        case ESC_CALIBRATION :
            Serial.println(F("ESC Calibration"));
            attitudeManager->setMaxPulse(); // Set maximum length pulse
            while(!receiver->throttleDown()){ // wait for the throttle to go down
                delay(100);
                receiver->run(); //Read receiver
            }
            Serial.println(F("Throttle Down"));
            attitudeManager->setMinPulse(); // Set minimum length pulse
            delay(1000); // wait for the ESC to copy correctly
            m_initState = IMU_INIT;
            break;


            // ESC Start : Wait for the long "beep". Go to IMU initialization
        case ESC_START :
            Serial.println(F("ESC Start"));
            delay(3000);
            m_initState = IMU_INIT;
            Serial.println(F("IMU Init"));
            break;


            //IMU initialization : Wait for the DMP to stabilize then reinitialize gyro path (in the function endInit())
            // Go to Fly state
        case IMU_INIT :
            sensors->run();
            Serial.print(sensors->eulerAngles[0]); Serial.print(F(","));
            Serial.print(sensors->eulerAngles[1]); Serial.print(F(","));
            Serial.print(sensors->eulerAngles[2]); Serial.print(F(","));
            Serial.print(sensors->gyroRates[0]); Serial.print(F(","));
            Serial.print(sensors->gyroRates[1]); Serial.print(F(","));
            Serial.println(sensors->gyroRates[2]);
            if(sensors->endInit())
            {
                m_initState = END_INIT;
                Serial.println(F("End Init"));
            }
            break;

        }
        delay(1);
    }
}

void Master::run()
{
    // Stateflow
    switch(m_state){

    case FLY_STATE :
        // Read RX
        float reference[4] = {0};
        float state[4] = {0};
        sensors->getAttitudeState(state);
        reference[0] = state[0] + receiver->getThrottleCommand(); // Here getTHrottleCOmmand gives a z increment
        reference[1] = receiver->getRollCommand();
        reference[2] = receiver->getPitchCommand();
        reference[3] = receiver->getYawCommand();

        // Set reference for attitudeManager
        attitudeManager->setReference(reference); //Update reference
        break;
    }
}
