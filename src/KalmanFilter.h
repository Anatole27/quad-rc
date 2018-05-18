#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <MatrixMath.h>

#define N_STATES 6
#define N_MEAS 6

class KalmanFilter
{
public:
    KalmanFilter();/*
    KalmanFilter(float* Phi, float* Q, float* H, float* P, float* R);

    void setPhi(float* Phi); // Set discete time state matrix
    void setQ(float* Q); // set Process noise
    void setH(float* H); // Set Measurement matrix
    void setP(float* P); // Set state covariance Matrix
    void setR(float* R); // Set Noise covariance matrix

    void getPhi(float* Phi);

    void update(float* state, float* meas);*/

private:
    float m_Phi[N_STATES][N_STATES]; // Discete state prediction matrix
    float m_PhiT[N_STATES][N_STATES]; // Transpose of Phi
    float m_Q[N_STATES][N_STATES]; // Process noise matrix
    float m_P[N_STATES][N_STATES]; // State Covariance matrix
    float m_H[N_MEAS][N_STATES]; // Measurement matrix
    /*float m_HT[N_STATES][N_MEAS]; // Measurement matrix transpose
    float m_R[N_MEAS][N_MEAS]; // Noise covariance matrix
    float m_K[N_STATES][N_MEAS]; // Kalman gain*/
};

#endif // KALMANFILTER_H


Out of memory mec !
Solution : Filter avec passe bas. Stou. De toute façon tes matrices sont toutes symétriques. Tu peux même faire le calcul toi-même : papier + stylo.
           Bon, moi je vais me mater le petit journal. Sois pas dèg !
