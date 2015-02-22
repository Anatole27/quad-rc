#ifndef UTIL_H
#define UTIL_H

class Util
{
public:
    Util();
    static void compareAttitudes(float* att1,float* att2, float* dest);
    static float modulo2pi(float angle);
    static float norm(float* vect, const int n);
    static void normalize(float* vect, const int n);
    static const float pi = 3.14159265359;
};

#endif // UTIL_H
