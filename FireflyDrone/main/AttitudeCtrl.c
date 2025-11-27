#include "quaternion.h"
#include "KalmanFilter.h"
#include "AttitudeCtrl.h"
#include <math.h>
#include "mpu6050.h"

#define PI 3.1415926535897932384

void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt) {
    Quaternion q_gyro = {0, gx, gy, gz};
    //运算
    Quaternion q_tnext = quatMultiply(*q, q_gyro);
    q_tnext.w *= 0.5;
    q_tnext.x *= 0.5;
    q_tnext.y *= 0.5;
    q_tnext.z *= 0.5;

    q->w += q_tnext.w * dt;
    q->x += q_tnext.x * dt;
    q->y += q_tnext.y * dt;
    q->z += q_tnext.z * dt;
    //归一
    quatNormalize(q);
}

void MEKF_filter(double dt, const double Q[3][3], const double R[3][3], mpu6050_raw_dat *raw) {
    MEKF filter;
    mekf_init(&filter);
    int ax_raw, ay_raw, az_raw;
    int gx_raw, gy_raw, gz_raw;
    double v_I[3] = {0, 0, 1};
    // read the data function
    ax_raw = raw -> ax;
    ay_raw = raw -> ay;
    az_raw = raw -> az;
    gx_raw = raw -> gx;
    gy_raw = raw -> gy;
    gz_raw = raw -> gz;
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
    // Unit in rads
    // I think the formula I typed in is correct
    roll = atan2(2 * (filter.q.x * filter.q.w + filter.q.y * filter.q.z), 1 - 2 * (filter.q.x * filter.q.x + filter.q.y * filter.q.y));
    pitch = - PI / 2 + 2 * atan2(sqrt(1 + 2 * (filter.q.w * filter.q.y - filter.q.x * filter.q.z)), sqrt(1 - 2 * (filter.q.w * filter.q.y - filter.q.x * filter.q.z)));
    yaw = atan2(2 * (filter.q.w * filter.q.z + filter.q.x * filter.q.y), 1 - 2 * (filter.q.y * filter.q.y + filter.q.z * filter.q.z));

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

void quatToEulerFloat(Quaternion q, float *roll, float *pitch, float *yaw) {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *roll = (float)(atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI));

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *pitch = (float)copysign(90.0, sinp);
    else
        *pitch = (float)(asin(sinp) * (180.0 / M_PI));

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    *yaw = (float)(atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI));
}