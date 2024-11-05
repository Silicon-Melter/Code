#include <util/atomic.h>
#include <Wire.h>
#include <MPU6050.h>  // Gyroscope library

MPU6050 mpu;

float target_heading = 0.0;
float current_heading = 0.0;
float left_distance, right_distance, distance_error;

// Integral and previous error terms for heading and centering
float integral_heading = 0, prev_error_heading = 0;
float integral_center = 0, prev_error_center = 0;

// Right Motor
const int IN2 = 9;    // Motor 2 direction pin 2
const int IN1 = 10;    // Motor 2 direction pin 1
const int ENA = 11;     // Motor 2 speed pin

// Left Motor
const int ENB = 6;     // Motor 1 speed pin
const int IN4 = 7;    // Motor 1 direction pin 2 actual 7
const int IN3 = 8;    // Motor 1 direction pin 1

float yaw = 0.0;
unsigned long lastYawUpdate = 0;

// PID constant
float kp_heading = 4, ki_heading = 0.05, kd_heading = 0.2;
float kp_center = 1.5, ki_center = 0.0, kd_center = 0.05;


void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);




  // Initialize gyroscope
  Wire.begin();
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  // Set initial heading as target
  //delayMillis(3000);
  //run(1);
}

void loop() {
  motor_left(155);
  motor_right(155);
 
}

void motor_left(int speed) {
  analogWrite(ENB, abs(speed));
  if (speed >= 0) {
    digitalWrite(IN3, LOW);  // Left motor forward
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
  }
}

void motor_right(int speed) {
  analogWrite(ENA, abs(speed));
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);  // Right motor forward
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  // Right motor backward
    digitalWrite(IN2, HIGH);
  }
}

