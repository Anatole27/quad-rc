#include "Master.h"

Master::Master()
{
    m_initState = INITIAL_STATE;
    m_state = FLY_STATE;
    wasYawCommandOverThr = false;
    for(int i = 0; i < 4; i++){
        reference[i] = 0;
    }
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
                delay(10);
                receiver->run(); //Read receiver
            }
            Serial.println(F("Throttle Down"));
            attitudeManager->setMinPulse(); // Set minimum length pulse
            delay(3000); // wait for the ESC to copy correctly
            //            for(int i = 0;i<4;i++)
            //                attitudeManager->testMotor(i);
            m_initState = IMU_INIT;
            Serial.println(F("IMU Init"));
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
            if(sensors->endInit())
            {
                m_initState = END_INIT;
            }
            break;

        }
        delay(1);
    }
    attitudeManager->enable();
}

void Master::run()
{
//    attitudeManager->getReference(reference);
    float state[4] = {0};

    //    sensors->alpha_angle = receiver->getChannel5()*(0.9-0.5)/100+0.5;
    //    sensors->alpha_gyro = receiver->getChannel6()*(0.9-0.5)/100+0.5;
    //    float Kp[4] = {0};
    //    float Kv[4] = {0};
    //    Kp[1] = receiver->getChannel5()*2*2044000.f/100;
    //    Kv[1] = receiver->getChannel6()*2*1164000.f/100;
    //    Kp[2] = Kp[1];
    //    Kv[2] = Kv[1];
    //    attitudeManager->setKPosition(Kp);
    //    attitudeManager->setKSpeed(Kv);

    // Stateflow
    switch(m_state){

    case FLY_STATE :
        sensors->getAttitudeState(state);
        // Read RX
        if(receiver->signalReceived()){ // Only if the TX signal is correctly received
            reference[0] = state[0] + receiver->getThrottleCommand(); // Here getTHrottleCOmmand gives a z increment
            reference[1] = receiver->getRollCommand();
            reference[2] = receiver->getPitchCommand();
            float yawCommand = receiver->getYawCommand();
            bool isYawCommandOverThr = (abs(yawCommand)*10 > YAWMAX);
            if(isYawCommandOverThr){ // Si la command est superieur a un seuil, on ajoute un delta par rapport au yaw
                reference[3] = state[3] + yawCommand;
            }
            else if(wasYawCommandOverThr){ // Si la commande vient de passer sous le seuil, on conserve state comme reference.
                reference[3] = state[3];
            }
            wasYawCommandOverThr = isYawCommandOverThr;
        }
        else // If error with signal, stay still
        {
            //Serial.println(F("Signal error"));
            Matrix.Copy(state,4,1,reference);
        }
        // Set reference for attitudeManager
        attitudeManager->setReference(reference); //Update reference

        if(receiver->throttleDown()){
            attitudeManager->disable();
            attitudeManager->setMinPulse();
            m_state = LANDING;
            Serial.println(F("Landing"));
        }
        break;

    case LANDING :
        if(!receiver->throttleDown()){
            attitudeManager->enable();
            m_state = FLY_STATE;
            Serial.println(F("Flight"));
        }
        break;
    }
}
