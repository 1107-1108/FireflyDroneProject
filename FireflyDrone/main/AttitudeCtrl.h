#ifndef ATTITUDECRTL_H
#define ATTITUDECRTL_H

#include "quaternion.h"
#include "KalmanFilter.h"
#include "AttitudeCtrl.h"
#include <math.h>

void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt);
Quaternion bodyToEarth(Quaternion q, Quaternion v_b);
double getAltitude(double pressure, double temperature);
void MEKF_filter(double dt, const double Q[3][3], const double R[3][3]);

#endif