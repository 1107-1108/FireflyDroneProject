#ifndef ATTITUDECRTL_H
#define ATTITUDECRTL_H

#include "quaternion.h"
#include "KalmanFilter.h"
#include "AttitudeCtrl.h"
#include <math.h>
#include "mpu6050.h"

#define GYRO_SENS 131.0        // for ±250°/s
#define ACCEL_SENS 16384.0         // for ±2g

void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt);
Quaternion bodyToEarth(Quaternion q, Quaternion v_b);
double getAltitude(double pressure, double temperature);
void MEKF_filter(double dt, const double Q[3][3], const double R[3][3], mpu6050_raw_dat *raw, double *roll, double *pitch, double *yaw);
void quatToEulerFloat(Quaternion q, float *roll, float *pitch, float *yaw);

#endif