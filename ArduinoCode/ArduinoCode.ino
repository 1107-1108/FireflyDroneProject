#include "MS5611.h"
#include <MPU6050_tockn.h>

int speed;

int motor1_speed;
int motor2_speed;
int motor3_speed;
int motor4_speed;

int MOTOR_1_PIN = 23;
int MOTOR_2_PIN = 22;
int MOTOR_3_PIN = 17;
int MOTOR_4_PIN = 16;

int MPU_PIN_SDA = 21;
int MPU_PIN_SCL = 19;

//MS5611 MS5611(0x77);
MPU6050 mpu6050(Wire);

class PID {
public:
  PID(double P, double I, double D)
    : Kp(P), Ki(I), Kd(D), integral(0), err(0), err_last(0), set_val(0) {}

  void set_point(double set_val) {
    this->set_val = set_val;
  }

  double PIDAlgorithm(double actual_val) {
    err = set_val - actual_val;
    integral += err;
    
    double output = Kp * err + Ki * integral + Kd * (err - err_last);
    err_last = err;
    return output;
  }

private:
  double Kp, Ki, Kd;
  double integral;
  double err, err_last;
  double set_val;
};

PID pid(1.0, 0.1, 0.01);

void PWM(int PWMPin, int PWMValue){  // 模拟PWM；频率100Hz
  PWMValue += 1000;
  digitalWrite(PWMPin, HIGH);
  delayMicroseconds(PWMValue);
  digitalWrite(PWMPin, LOW);
  delayMicroseconds(10000 - PWMValue);
}

void MOTOR_INIT(int motorPins[], int size) {
  // 好盈电调初始化
  for(int i = 0; i <= 55; ++i) {
    for (int j = 0; j < size; ++j) {
      PWM(motorPins[j], 1000);
    }
  }
  for (int i = 0; i <= 100; ++i) {
    for (int j = 0; j < size; ++j) {
      PWM(motorPins[j], 0); //油门最小
    }
  }
  delay(2000);
}

float calculateAltitude(float currentPressure, float currentTemperature) {
  const float P0 = 101325;
  currentPressure = currentPressure * 100;
  float height = ((pow((P0 / currentPressure), (1 / 5.257)) - 1) * (currentTemperature + 273.15)) / 0.0065;
  return height;
}

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float &roll, float &pitch, float &yaw) {
  static float dt = 0.01;
  static unsigned long lastTime = 0;
  static float alpha = 0.95; // 互补滤波系数

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // 转换为秒
  lastTime = currentTime;

  // 加速度计计算roll和pitch
  float accRoll = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // 陀螺仪计算角度增量
  float gyroRollRate = gx;
  float gyroPitchRate = gy;
  float gyroYawRate = gz;

  // 积分得到角度
  roll += gyroRollRate * dt;
  pitch += gyroPitchRate * dt;
  yaw += gyroYawRate * dt;

  // 互补滤波融合
  roll = alpha * roll + (1.0 - alpha) * accRoll;
  pitch = alpha * pitch + (1.0 - alpha) * accPitch;
}

float roll, pitch, yaw = 0;

float ax, ay, az, gx, gy, gz;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

const float Kp_IMU = 25;
const float Ki_IMU = 0.0355;
float exInt = 0, eyInt = 0, ezInt = 0;
float norm;


void IMU_update_quer(float gx, float gy, float gz, float ax, float ay, float az) {
  float vx, vy, vz;
  float ex, ey, ez;

  norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  vx = 2 * (q1 * q3 - q0 * q2);
  vy = 2* (q0 * q1 + q2 * q3);
  vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  exInt = exInt + ex * Ki_IMU;
  eyInt = eyInt + ey * Ki_IMU;
  ezInt = ezInt + ez * Ki_IMU;

  gx = gx + Kp_IMU * ex + exInt;
  gy = gy + Kp_IMU * ey + eyInt;
  gz = gz + Kp_IMU * ez + ezInt;

  q0 = q0 + 0.001 * (-q1 * gx - q2 * gy - q3 * gz);
  q1 = q1 + 0.001 * (q0 * gx + q2 * gz - q3 * gy);
  q2 = q2 + 0.001 * (q0 * gy - q1 * gz + q3 * gx);
  q3 = q3 + 0.001 * (q0 * gz + q1 * gy - q2 * gx);

  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  pitch = sin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
  roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
}


void get_roll(float angle) {
  pid.set_point(angle);
  double control_signal = pid.PIDAlgorithm(pitch);
  Serial.print("Control_Signal: ");
  Serial.println(control_signal);

  int base_speed = 200;
  motor1_speed = base_speed + control_signal;
  motor2_speed = base_speed - control_signal;
  motor3_speed = base_speed + control_signal;
  motor4_speed = base_speed - control_signal;

  motor1_speed = constrain(motor1_speed, 0, 400);
  motor2_speed = constrain(motor1_speed, 0, 400);
  motor3_speed = constrain(motor1_speed, 0, 400);
  motor4_speed = constrain(motor1_speed, 0, 400);

  PWM(MOTOR_1_PIN, motor1_speed);
  PWM(MOTOR_2_PIN, motor2_speed);
  PWM(MOTOR_3_PIN, motor3_speed);
  PWM(MOTOR_4_PIN, motor4_speed);
  
}


  

void setup() {
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);
  pinMode(MPU_PIN_SDA, OUTPUT);
  pinMode(MPU_PIN_SCL, OUTPUT);


  Serial.begin(115200);
  Serial.println();
  Serial.println("Hello");
  Wire.begin(MPU_PIN_SDA, MPU_PIN_SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);


  int motorPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};
  MOTOR_INIT(motorPins, 4);

  


  //GY63 高度计初始化
  // Wire.begin(21,22);
  //GY63 初始化结束

  

}

void loop() {
  mpu6050.update();
  //mpu6050读数
  ax = mpu6050.getAccX();
  ay = mpu6050.getAccY();
  az = mpu6050.getAccZ();
  gx = mpu6050.getGyroX();
  gy = mpu6050.getGyroY();
  gz = mpu6050.getGyroZ();
  
  IMU_update_quer(gx, gy, gz, ax, ay, az);
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

  //get_roll(0);

  /*
  // 给他一个转速
  speed = 100;

  motor1_speed = speed;
  motor2_speed = speed;
  motor3_speed = speed;
  motor4_speed = speed;

  PWM(MOTOR_1_PIN, motor1_speed);
  PWM(MOTOR_2_PIN, motor2_speed);
  PWM(MOTOR_3_PIN, motor3_speed);
  PWM(MOTOR_4_PIN, motor4_speed);
  */

  /*
  MS5611.read();
  float height = calculateAltitude(MS5611.getPressure(), MS5611.getTemperature());
  */
}