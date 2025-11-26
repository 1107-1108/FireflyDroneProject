#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

typedef struct {
    double w;
    double x;
    double y;
    double z;
} Quaternion;

Quaternion quatMultiply(Quaternion q1, Quaternion q2);

void quatNormalize(Quaternion *q);

Quaternion quatConjugate(Quaternion q);


#endif