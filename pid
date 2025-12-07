#include <stdio.h>

typedef struct{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_measurement;
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max,
    float integral_min, float integral_max){
        pid -> kp = kp;
        pid -> ki = ki;
        pid -> kd = kd;

        pid -> integral = 0.0f;
        pid -> prev_measurement = 0.0f;

        pid -> out_min = out_min;
        pid -> out_max = out_max;
        pid -> integral_min = integral_min;
        pid -> integral_max = integral_max;

}
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{   
    if (dt <= 0.0f) return 0.0f;
    float error = setpoint - measurement;

    float p_term = pid -> kp * error;

    pid->integral += error * dt;
    if (pid -> integral > pid -> integral_max) pid -> integral = pid -> integral_max;
    if (pid -> integral < pid -> integral_min) pid -> integral = pid -> integral_min;
    float i_term = pid -> ki * pid -> integral;

    float derivative = (error - pid -> prev_error) / dt;
    float d_term = pid -> kd * derivative;

    pid->prev_error = error;

    float output = p_term + i_term + d_term;

    if (output > pid -> out_max) output = pid -> out_max;
    if (output < pid -> out_min) output = pid -> out_min;

    return output;
}

    PID_t pid_roll;
    PID_t pid_pitch;
    PID_t pid_yaw;

    void attitude_pid_init(void){
        PID_Init(&pid_roll, 4.0f, 0.02f, 0.8f, -200.0f, 200.0f, -50, 50);
        PID_Init(&pid_pitch, 4.0f, 0.02f, 0.8f, -200.0f, 200.0f, -50, 50);
        PID_Init(&pid_yaw, 2.0f, 0.01f, 0.5f, -200.0f, 200.0f, -50, 50);
    }

