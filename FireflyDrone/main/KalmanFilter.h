#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <stdbool.h>
#include <stdio.h>
#include "quaternion.h"

typedef struct {
    Quaternion q;
    double P[3][3];
} MEKF;

void cross_product(const double a[3], const double b[3], double res[3]);
void skew(const double vec[3], double res[3][3]);
bool matrix3_inverse(const double A[3][3], double invA[3][3]);
void mekf_init(MEKF *model);
void mekf_predict(MEKF *model, const double omega[3], double dt, const double Q[3][3]);
void mekf_update(MEKF *model, const double z[3], const double v_I[3], const double R[3][3]);
void MEKF_step(MEKF *model, const double omega[3], double dt, const double z[3], const double v_I[3], const double Q[3][3], const double R[3][3]);


#endif