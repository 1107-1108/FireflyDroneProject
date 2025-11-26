#include <math.h>
#include "quaternion.h"

Quaternion quatMultiply(Quaternion q1, Quaternion q2) {
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

void quatNormalize(Quaternion *q) {
    double norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

Quaternion quatConjugate(Quaternion q) {
    Quaternion res = q;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}