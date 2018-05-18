#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    while(true){
        Serial.println(F("Coucou"));
        delay(100);
    }
   /*Matrix.Identity((float*)m_Phi,N_STATES);
    Matrix.Identity((float*)m_Q,N_STATES);
    Matrix.Init((float*)m_H,N_MEAS,N_STATES,0);
    Matrix.Init((float*)m_K,N_STATES,N_MEAS,0);
    Matrix.Identity((float*)m_P,N_STATES);
    Matrix.Identity((float*)m_R,N_MEAS);*/
}/*

KalmanFilter::KalmanFilter(float* Phi, float* Q, float* H, float* P, float* R)
{
    while(true){
        Serial.println(F("Coucou"));
        delay(100);
    }
    Matrix.Copy((float*)Phi,N_STATES,N_STATES,(float*)m_Phi);
    Matrix.Copy((float*)Q,N_STATES,N_STATES,(float*)m_Q);
    Matrix.Copy((float*)H,N_MEAS,N_STATES,(float*)m_H);
    Matrix.Copy((float*)P,N_STATES,N_STATES,(float*)m_P);
    Matrix.Copy((float*)R,N_MEAS,N_MEAS,(float*)m_R);
    Matrix.Init((float*)m_K,N_STATES,N_MEAS,0);
}

void KalmanFilter::setPhi(float* Phi)
{
    Matrix.Copy((float*)Phi,N_STATES,N_STATES,(float*)m_Phi);
}

void KalmanFilter::setQ(float* Q)
{
    Matrix.Copy((float*)Q,N_STATES,N_STATES,(float*)m_Q);
}

void KalmanFilter::setH(float* H)
{
    Matrix.Copy((float*)H,N_STATES,N_STATES,(float*)m_H);
}

void KalmanFilter::setP(float* P)
{
    Matrix.Copy((float*)P,N_MEAS,N_STATES,(float*)m_P);
}

void KalmanFilter::setR(float* R)
{
    Matrix.Copy((float*)R,N_MEAS,N_MEAS,(float*)m_R);
}

void KalmanFilter::getPhi(float* Phi)
{
    Matrix.Copy((float*)m_Phi,N_STATES,N_STATES,(float*)Phi);
}

void KalmanFilter::update(float *state, float *meas)
{
    // Predict
    float statePlus1[N_STATES] = {0};
    Matrix.Multiply((float*)m_Phi,(float*)state,N_STATES,N_STATES,1,(float*)statePlus1); // statePlus1 = Phi*state

    float PhiP[N_STATES*N_STATES] = {0};
    float PPlus1[N_STATES*N_STATES] = {0};
    Matrix.Multiply((float*)m_Phi,(float*)m_P,N_STATES,N_STATES,N_STATES,(float*)PhiP);
    Matrix.Multiply((float*)PhiP,(float*)m_PhiT,N_STATES,N_STATES,N_STATES,(float*)PPlus1); // PPlus1 = Phi*P*Phi^T

    // Measure
    float HStatePlus1[N_MEAS] = {0};
    float innov[N_MEAS] = {0};
    Matrix.Multiply((float*)m_H,(float*)statePlus1,N_MEAS,N_STATES,1,(float*)HStatePlus1);
    Matrix.Subtract((float*)meas,(float*)HStatePlus1,N_MEAS,1,(float*)innov); // innov = meas - H*statePlus1

    float S[N_MEAS*N_MEAS] = {0};
    float HPplus1HT[N_MEAS*N_MEAS] = {0};
    Matrix.Multiply((float*)m_H,(float*)PPlus1,N_MEAS,N_STATES,N_STATES,(float*)S);
    Matrix.Multiply((float*)S,(float*)m_HT,N_STATES,N_STATES,N_MEAS,(float*)HPplus1HT);
    Matrix.Add((float*)HPplus1HT,(float*)m_R,N_MEAS,N_MEAS,(float*)S); // S = H*PPlus1*H^T + R

    float HTSinv[N_STATES*N_MEAS] = {0};
    Matrix.Invert((float*)S,N_MEAS); // S <- S^-1
    Matrix.Multiply((float*)m_HT,(float*)S,N_STATES,N_MEAS,N_MEAS,(float*)HTSinv);
    Matrix.Multiply((float*)PPlus1,(float*)HTSinv,N_STATES,N_STATES,N_MEAS,(float*)m_K); // K = PPlus1*H^T*S^-1

    // Correction
    float Kinnov[N_STATES] = {0};
    Matrix.Multiply((float*)m_K,(float*)innov,N_STATES,N_MEAS,1,(float*)Kinnov);
    Matrix.Add((float*)statePlus1,(float*)Kinnov,N_STATES,1,(float*)state); // state = statePlus1 + K*innov

    float ident[N_STATES*N_STATES] = {0};
    Matrix.Identity((float*)ident,N_STATES);
    float I_KH[N_STATES*N_STATES] = {0};
    Matrix.Multiply((float*)m_K,(float*)m_H,N_STATES,N_MEAS,N_STATES,(float*)PhiP);
    Matrix.Subtract((float*)ident,(float*)PhiP,N_STATES,N_STATES,(float*)I_KH);
    Matrix.Multiply((float*)I_KH,(float*)PPlus1,N_STATES,N_STATES,N_STATES,(float*)m_P); // P = (I - KH)*PPlus1
}
*/
