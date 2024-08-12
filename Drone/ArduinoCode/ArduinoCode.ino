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

int GY_PIN_SDA = 27;
int GY_PIN_SCL = 26;

MS5611 MS5611(0x77);
MPU6050 mpu6050(Wire1);

char comdata[50];
long previousMillis = millis();     //上一次激活时间
long interval = 300; 


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

PID pid(2.0, 0.1, 0.01);


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



float roll = 0, pitch = 0, yaw = 0;


float ax, ay, az, gx, gy, gz;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

const float Kp_IMU = 26;
const float Ki_IMU = 0.0355;
float exInt = 0, eyInt = 0, ezInt = 0;
float norm;

const int SAMPLE_SIZE = 20;
int sampleIndex = 0;
float rollBuffer[SAMPLE_SIZE] = {0.0};
float pitchBuffer[SAMPLE_SIZE] = {0.0};
float current_roll, current_pitch;
int exp_roll, exp_pitch, exp_yaw;
float exp_altitude;
int up_num, down_num;

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

  current_pitch = sin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
  current_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

  rollBuffer[sampleIndex] = current_roll;
  pitchBuffer[sampleIndex] = current_pitch;

  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;

  if (sampleIndex == 0) {
    float rollSum = 0.0;
    float pitchSum = 0.0;
    for (int i = 0 ; i < SAMPLE_SIZE; ++i) {
      rollSum += rollBuffer[i];
      pitchSum += pitchBuffer[i];
    }

    roll = rollSum / SAMPLE_SIZE;
    pitch = pitchSum / SAMPLE_SIZE;
  }
}

PID pid_roll(2.0, 0.1, 0.01);
PID pid_pitch(2.0, 0.1, 0.01);
PID pid_yaw(2.0, 0.1, 0.01);
PID pid_altitude(2.0, 0.1, 0.01);

float current_altitude;
float target_altitude = 0.0;



void update_motor_speeds(float exp_roll, float exp_pitch, float exp_yaw, float exp_altitude) {

  mpu6050.update();
  ax = mpu6050.getAccX();
  ay = mpu6050.getAccY();
  az = mpu6050.getAccZ();
  gx = mpu6050.getGyroX();
  gy = mpu6050.getGyroY();
  gz = mpu6050.getGyroZ();
  IMU_update_quer(gx, gy, gz, ax, ay, az);

  MS5611.read();
  current_altitude = calculateAltitude(MS5611.getPressure(), MS5611.getTemperature());

  pid_roll.set_point(exp_roll);
  pid_pitch.set_point(exp_pitch);
  pid_yaw.set_point(exp_yaw);
  pid_altitude.set_point(exp_altitude / 90 + current_altitude);


  double roll_control_signal = pid_roll.PIDAlgorithm(roll);
  double pitch_control_signal = pid_pitch.PIDAlgorithm(pitch);
  double yaw_control_signal = pid_yaw.PIDAlgorithm(yaw);
  double altitude_control_signal = pid_altitude.PIDAlgorithm(current_altitude);

  int base_speed = 150;
  motor1_speed = base_speed + roll_control_signal + pitch_control_signal - yaw_control_signal + altitude_control_signal;
  motor2_speed = base_speed - roll_control_signal + pitch_control_signal + yaw_control_signal + altitude_control_signal;
  motor3_speed = base_speed + roll_control_signal - pitch_control_signal + yaw_control_signal + altitude_control_signal;
  motor4_speed = base_speed - roll_control_signal - pitch_control_signal - yaw_control_signal + altitude_control_signal;

  motor1_speed = constrain(motor1_speed, 0, 1000);
  motor2_speed = constrain(motor2_speed, 0, 1000);
  motor3_speed = constrain(motor3_speed, 0, 1000);
  motor4_speed = constrain(motor4_speed, 0, 1000);

  PWM(MOTOR_1_PIN, motor1_speed);
  PWM(MOTOR_2_PIN, motor2_speed);
  PWM(MOTOR_3_PIN, motor3_speed);
  PWM(MOTOR_4_PIN, motor4_speed);

}

void processData(String comdata, int &exp_pitch, int &exp_roll, int &exp_yaw, int &other) {
  if (comdata.startsWith("Joystick:")) {
    int colonIndex = comdata.indexOf(':');
    if (colonIndex != -1) {
      String numbersPart = comdata.substring(colonIndex + 1);
      numbersPart.trim();

      int firstComma = numbersPart.indexOf(',');
      int secondComma = numbersPart.indexOf(',', firstComma + 1);
      int thirdComma = numbersPart.indexOf(',', secondComma + 1);

      exp_pitch = numbersPart.substring(0, firstComma).toInt(); // pitch
      exp_roll = numbersPart.substring(firstComma + 1, secondComma).toInt(); // roll
      exp_yaw = numbersPart.substring(secondComma + 1, thirdComma).toInt(); // yaw
      exp_altitude = numbersPart.substring(thirdComma + 1).toFloat(); // altitude
    }
  }
}

  

void setup() {
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);
  pinMode(MPU_PIN_SDA, OUTPUT);
  pinMode(MPU_PIN_SCL, OUTPUT);


  Serial.begin(9600);
  Serial2.begin(9600,SERIAL_8N1,32, 33);
  Serial.println();
  Serial.println("Hello");
  Wire1.begin(MPU_PIN_SDA, MPU_PIN_SCL);
  Wire.begin(GY_PIN_SDA, GY_PIN_SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);


  int motorPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};
  MOTOR_INIT(motorPins, 4);
  MS5611.begin();
  exp_altitude = calculateAltitude(MS5611.getPressure(), MS5611.getTemperature());
}

void send_data(String msg) {
  Serial2.print(msg);
}


void Receive_Data() {
  char endChar = '\n';
  int index = 0;

  while (Serial2.available() > 0 && index < sizeof(comdata) - 1) {
    char incomingChar = char(Serial2.read());

    if (incomingChar == endChar) {
      break;
    }

    comdata[index++] = incomingChar;
    delay(2);
  }
  comdata[index] = '\0';

  if (index > 0) {
    Serial.print("Received comdata: ");
    Serial.print(comdata);
    processData(comdata, exp_pitch, exp_roll, up_num, down_num);
  }
}


const unsigned long updateInterval = 30;
unsigned long previousUpdateTime = 0;

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousUpdateTime >= updateInterval) {
    previousUpdateTime = currentMillis;

    if (Serial2.available()) {
      Receive_Data();
    }


    update_motor_speeds(exp_roll, exp_pitch, exp_yaw, exp_altitude);

    mpu6050.update();
    ax = mpu6050.getAccX();
    ay = mpu6050.getAccY();
    az = mpu6050.getAccZ();
    gx = mpu6050.getGyroX();
    gy = mpu6050.getGyroY();
    gz = mpu6050.getGyroZ();

    MS5611.read();
    float height = calculateAltitude(MS5611.getPressure(), MS5611.getTemperature());
    Serial.println(height);

    IMU_update_quer(gx, gy, gz, ax, ay, az);

    Serial.print("exp_pitch");
    Serial.print(exp_pitch);
    Serial.print("exp_roll");
    Serial.print(exp_roll);
    Serial.print("exp_altitude");
    Serial.print(exp_altitude);
    Serial.print("exp_yaw");
    Serial.println(exp_yaw);
  }
}
