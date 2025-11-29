#include <stdbool.h>
#include <stdio.h>
#include "quaternion.h"
#include "KalmanFilter.h"

//cross product of vector
void cross_product(const double a[3], const double b[3], double res[3]) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

void skew(const double vec[3], double res[3][3]) {
    res[0][0] = 0;
    res[0][1] = -vec[2];
    res[0][2] = vec[1];
    res[1][0] = vec[2];
    res[1][1] = 0;
    res[1][2] = -vec[0];
    res[2][0] = -vec[1];
    res[2][1] = vec[0];
    res[2][2] = 0;
}

bool matrix3_inverse(const double A[3][3], double invA[3][3]) {
    double det =
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabs(det) < 1e-12)
        return false;  // cannot inverse

    double invDet = 1.0 / det;

    invA[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    invA[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    invA[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

    invA[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    invA[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    invA[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

    invA[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    invA[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    invA[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return true;
}

void errvec2quat(const double vec[3], Quaternion *dquat) {
    double theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (theta < 1e-8) {
        dquat->x = 0.5 * vec[0];
        dquat->y = 0.5 * vec[1];
        dquat->z = 0.5 * vec[2];
        dquat->w = 1.0;
    } else {
        dquat->x = vec[0] * (sin(theta/2.0) / theta);
        dquat->y = vec[1] * (sin(theta/2.0) / theta);
        dquat->z = vec[2] * (sin(theta/2.0) / theta);
        dquat->w = cos(theta / 2.0);
    }
    quatNormalize(dquat);
}

void mekf_init(MEKF *model) {
    model -> q.x = 0;
    model -> q.y = 0;
    model -> q.z = 0;
    model -> q.w = 1;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            model -> P[i][j] = (i == j) ? 1e-4 : 0; // Adjust the parameter~
        }
    }
}

void mekf_predict(MEKF *model, const double omega[3], double dt, const double Q[3][3]) {
    double phi[3] = {omega[0]*dt, omega[1]*dt, omega[2]*dt};
    Quaternion dquat;
    Quaternion nquat;


    errvec2quat(phi, &dquat);
    //update quats
    nquat = quatMultiply(dquat, model->q);
    
    model->q.x = nquat.x;
    model->q.y = nquat.y;
    model->q.z = nquat.z;
    model->q.w = nquat.w;

    //update cov
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            model->P[i][j] += Q[i][j];
        }
    }
}

void mekf_update(MEKF *model, const double z[3], const double v_I[3], const double R[3][3]) {
    // Rotation Matrix from quaternion
    double qx = model->q.x;
    double qy = model->q.y;
    double qz = model->q.z;
    double qw = model->q.w;

    double A[3][3] = {
        {1 - 2*(qy*qy + qz*qz),  2*(qx*qy - qz*qw),      2*(qx*qz + qy*qw)},
        {2*(qx*qy + qz*qw),      1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)},
        {2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),      1 - 2*(qx*qx + qy*qy)}
    };

    // Predict measurement vB = A * v_I
    double vB[3];
    for (int i = 0; i < 3; ++i) {
        vB[i] = A[i][0]*v_I[0] + A[i][1]*v_I[1] + A[i][2]*v_I[2];
    }

    // Innovation y = z - vB
    double y[3];
    for (int i = 0; i < 3; ++i) y[i] = z[i] - vB[i];

    // H = -[vB]_x  (skew symmetric)
    double H[3][3];
    skew(vB, H);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            H[i][j] = -H[i][j];

    // Compute PHt = P * H^T  (3x3)
    double PHt[3][3] = {{0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { // column of H^T -> row of H
            for (int k = 0; k < 3; ++k) {
                PHt[i][j] += model->P[i][k] * H[j][k];
            }
        }
    }

    // Compute S = H * PHt + R   (3x3)
    double S[3][3] = {{0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            S[i][j] += R[i][j];
        }
    }

    // Invert S
    double invS[3][3];
    if (!matrix3_inverse(S, invS)) {
        printf("WARNING:: S inverse failed, skip update\n");
        return;
    }

    // Compute K = PHt * invS   (3x3)
    double K[3][3] = {{0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                K[i][j] += PHt[i][k] * invS[k][j];
            }
        }
    }

    // a = K * y  (3x1)  — attitude error vector in small-angle form
    double a[3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            a[i] += K[i][j] * y[j];

    // Attitude correction: integrate small error into quaternion
    Quaternion dquat, nquat;
    errvec2quat(a, &dquat);
    nquat = quatMultiply(dquat, model->q);
    model->q = nquat;
    quatNormalize(&model->q);

    // Update covariance P using Joseph form: P = (I-KH) P (I-KH)^T + K R K^T
    double IminusKH[3][3];
    // compute KH first for convenience
    double KH[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                KH[i][j] += K[i][k] * H[k][j];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            IminusKH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];

    // temp = (I-KH) * P
    double temp[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                temp[i][j] += IminusKH[i][k] * model->P[k][j];

    // P_new = temp * (I-KH)^T
    double P_new[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                P_new[i][j] += temp[i][k] * IminusKH[j][k];

    // Add K*R*K^T term
    double KR[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                KR[i][j] += K[i][k] * R[k][j];

    double KRKT[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                KRKT[i][j] += KR[i][k] * K[j][k];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            model->P[i][j] = P_new[i][j] + KRKT[i][j];
}

void MEKF_step(MEKF *model, const double omega[3], double dt, const double z[3], const double v_I[3], const double Q[3][3], const double R[3][3]) {
    mekf_predict(model, omega, dt, Q);
    mekf_update(model, z, v_I, R);
}