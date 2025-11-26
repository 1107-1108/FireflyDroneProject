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

void update(MEKF *model, const double z[3], const double v_I[3], const double R[3][3]) {
    // Rotation Matrix
    double qx, qy, qz, qw;
    qx = model->q.x;
    qy = model->q.y;
    qz = model->q.z;
    qw = model->q.w;
    double A[3][3] = {
        {1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)},
        {2*(qx*qy + qz*qw), 1-  2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)},
        {2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)}
    };  // quat to roration matrix

    //Predict Measurement
    // V_B = A(q_ref) * V_I
    double vB[3];
    vB[0] = A[0][0]*v_I[0] + A[0][1]*v_I[1] + A[0][2]*v_I[2];
    vB[1] = A[1][0]*v_I[0] + A[1][1]*v_I[1] + A[1][2]*v_I[2];
    vB[2] = A[2][0]*v_I[0] + A[2][1]*v_I[1] + A[2][2]*v_I[2];

    //Err between measure and predict
    // y = z - v_B
    double y[3];
    y[0] = z[0] - vB[0];
    y[1] = z[1] - vB[1];
    y[2] = z[2] - vB[2];

    // H = -[vB]_x
    double H[3][3];
    skew(vB, H);
    for(int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H[i][j] = -H[i][j];
        }
    }
    
    //Kalman gain
    //K = PH^T(HPH^T + R)^-1
    //S = HPH^T + R
    double S[3][3] = {{0}};
    for (int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                S[i][j] += H[i][k] * model->P[k][0] * H[j][0] + H[i][k] * model->P[k][1] * H[j][1] + H[i][k] * model-> P[k][2] * H[j][2];  
            }
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] += R[i][j];
        }
    }
    //S^-1
    double invS[3][3];
    if (!matrix3_inverse(S, invS)) {
        printf("WARNING:: inverse uncessfully, skip update \n");
        return;
    }
    //K = PH^T S^-1
    double K[3][3] = {{0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for(int k = 0; k < 3; ++k) {
                K[i][j] += model->P[i][k] * H[j][k] * invS[j][j];
            }
        }
    }
    // a = Ky
    double a[3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i] += K[i][j] * y[j];
        }
    }

    //Attitude Fix
    Quaternion dquat;
    Quaternion nquat;
    errvec2quat(a, &dquat);
    nquat = quatMultiply(dquat, model->q);
    model->q.x = nquat.x;
    model->q.y = nquat.y;
    model->q.z = nquat.z;
    model->q.w = nquat.w;
    quatNormalize(&model->q);
}

void MEKF_step(MEKF *model, const double omega[3], double dt, const double z[3], const double v_I[3], const double Q[3][3], const double R[3][3]) {
    mekf_predict(model, omega, dt, Q);
    update(model, z, v_I, R);
}