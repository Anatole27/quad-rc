#include "Util.h"
#include "Arduino.h"

Util::Util()
{
}

void Util::compareAttitudes(float *att1, float *att2, float *dest)
{
    dest[0] = att1[0]-att2[0];
    for(int i=1;i<4;i++)
    {
        dest[i] = modulo2pi(att1[i]-att2[i]);
    }
}

float Util::modulo2pi(float angle)
{
    return fmod(angle+PI,2*PI)-PI;
}

float Util::norm(float* vect, const int n)
{
    float norm_ = 0;
    for(int i=0;i<n;i++)
    {
        norm_ += vect[i]*vect[i];
    }
    norm_ = sqrt(norm_);

    return norm_;
}

void Util::normalize(float* vect, const int n)
{
    float norm_vect = norm(vect,n);
    if(norm_vect != 0)
    {
        for(int i=0;i<n;i++)
        {
            vect[i] = vect[i]/norm_vect;
        }
    }
}
