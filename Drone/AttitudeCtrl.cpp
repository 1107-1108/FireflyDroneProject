#include "quaternion.h"
#include "KalmanFilter.h"
#include "AttitudeCtrl.h"

void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt) {
    Quaternion q_gyro = {0, gx, gy, gz};
    //运算
    Quaternion q_tnext = quatMultiply(*q, q_gyro);
    q_tnext.w *= 0.5;
    q_tnext.x *= 0.5;
    q_tnext.y *= 0.5;
    q_tnext.z *= 0.5;

    q_gyro.w += q_tnext.w * dt;
    q_gyro.x += q_tnext.x * dt;
    q_gyro.y += q_tnext.y * dt;
    q_gyro.z += q_tnext.z * dt;
    //归一
    quatNormalize(q);
}

void MEKF_filter(double dt, const double Q[3][3], const double R[3][3]) {
    MEKF filter;
    mekf_init(&filter);
    int ax_raw, ay_raw, az_raw;
    int gx_raw, gy_raw, gz_raw;
    double v_I[3] = {0, 0, 1};
    // read the data function
    // mpu6050_r(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    double omega[3] = {
        // TODO
        // Angle Transfer
        // Well idk how to do it so its not my work lmao
        // QwQ......
    };
    double accel[3] = {
        // TODO
        // Dont know how to do as well 
        // QAQ.......
    };
    mekf_predict(&filter, omega, dt, Q);
    double acc_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    double z[3] = { accel[0]/acc_norm, accel[1]/acc_norm, accel[2]/acc_norm };
    mekf_update(&filter, z, v_I, R);
    double roll, pitch, yaw;
    // TODO :: A quat to euler function here
}

Quaternion bodyToEarth(Quaternion q, Quaternion v_b) {
    //q 机体姿态, v_b 机体坐标系
    Quaternion q_con = quatConjugate(q);
    Quaternion res = quatMultiply(quatMultiply(q, v_b), q_con);
    return res;
}

double getAltitude(double pressure, double temperature) {
    // Pressures(Pa)
    // Temperature(Celcius)

    double temp_K = temperature + 273.15;
    double ratio = pressure / 101325.0;
    double altitude = (temp_K / 0.0065) * (1 - pow(ratio, (287.05 * 0.0065 / 9.80665)));
    return altitude;
}