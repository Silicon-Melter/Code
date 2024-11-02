#include <util/atomic.h>
#include <Wire.h>
#include <MPU6050.h>  // Gyroscope library

MPU6050 mpu;

float target_heading = 0.0;
float current_heading = 0.0;
float left_distance, right_distance, distance_error;

// PID gains for heading and centering
float kp_heading = 4, ki_heading = 0.05, kd_heading = 0.2;
float kp_center = 1.0, ki_center = 0.0, kd_center = 0.0;

// Integral and previous error terms for heading and centering
float integral_heading = 0, prev_error_heading = 0;
float integral_center = 0, prev_error_center = 0;

// Right Motor
int encoder2_pin = 2;  // Encoder 2 pin actual 2
const int IN2 = 10;    // Motor 2 direction pin 2
const int IN1 = 11;    // Motor 2 direction pin 1
const int ENA = 5;     // Motor 2 speed pin

// Left Motor
int encoder1_pin = 3;  // Encoder 1 pin default 1
const int ENB = 6;     // Motor 1 speed pin
const int IN4 = 12;    // Motor 1 direction pin 2 actual 7
const int IN3 = 13;    // Motor 1 direction pin 1

volatile byte pulses1;  // number of pulses for encoder 1
volatile byte pulses2;  // number of pulses for encoder 2
unsigned long timeold;
unsigned int pulsesperturnleft = 53;
unsigned int pulsesperturnright = 53;
float wheel_diameter = 67;             // Diameter of the wheel in mm
float wheel_circum = (67 / 2) * 6.28;  // Circumference of the wheel

#define left_ultra A0
#define front_ultra A1
#define right_ultra A2

float timeStep = 0.01;
unsigned long timer = 0;
float yaw = 0.0;
unsigned long lastYawUpdate = 0;

void counter1() {
  pulses1++;
}

void counter2() {
  pulses2++;
}


void setup() {
  Serial.begin(9600);
  pinMode(encoder1_pin, INPUT);
  pinMode(encoder2_pin, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(left_ultra, INPUT);
  pinMode(front_ultra, INPUT);
  pinMode(right_ultra, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
  pulses1 = 0;
  pulses2 = 0;
  timeold = millis();
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
  target_heading = getGyroHeading();
  // Set initial heading as target
  delayMillis(3000);
  driveStraightWithPID(150.0, 155); 
}

void loop() {
  /* delayMillis(3000);
  updateYaw(); 
  Serial.print("set traget");
  Serial.println(target_heading);
  motor_left(155);
  motor_right(-155);
  delayMillis(1000);
  motor_left(0);
  motor_right(0);
  //read_error();
  delayMillis(100); */
  //reset_to_initial_heading(155);
  
  /* motor_right(155);
    motor_left(-155); */
  /* Serial.println("move robot");
    delay(3000);
    Serial.println("adjusting robot");
    delay(1000);
    reset_to_initial_heading(155); */
  /* analogWrite(ENB, 155);
    analogWrite(ENA, 155);
    digitalWrite(IN3, LOW); // Left motor forward
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, HIGH); // Right motor forward
    digitalWrite(IN2, LOW); */
}

void delayMillis(unsigned long delayTime) {
    unsigned long startTime = millis();
    while (millis() - startTime < delayTime) {
        // Insert any code here that needs to run while waiting
        
        updateYaw();  
    }
}

void updateYaw() {
    unsigned long currentTime = millis();
    float deltaT = (currentTime - lastYawUpdate) / 1000.0;  // Calculate time difference in seconds
    lastYawUpdate = currentTime;

    // Read normalized gyroscope data
    Vector norm = mpu.readNormalizeGyro();

    // Update yaw using the Z-axis rotation rate
    yaw += norm.ZAxis * deltaT;
}

// Modify getGyroHeading to simply return the updated yaw
float getGyroHeading() {
    updateYaw();
    return yaw;
}

void read_error() {
  float heading_error;
  while (true) {
    updateYaw();
    current_heading = getGyroHeading();
    heading_error = target_heading - current_heading;
    Serial.println(heading_error);
    if (heading_error > 1.0) {
      Serial.println("turning left");
    } else if (heading_error < -1.0) {
      Serial.println("turning right");
    }
    if (abs(heading_error) < 1.0) {
      break;
    }
  }
  Serial.println("Stop");
}

float pidController(float error, float& integral, float& prev_error, float kp, float ki, float kd, float deltaT) {
  integral += error * deltaT;
  float derivative = (error - prev_error) / deltaT;
  prev_error = error;
  return kp * error + ki * integral + kd * derivative;
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

void driveStraightWithPID(float threshold, int base_speed) {
  
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = analogRead(front_ultra);
    if (front_distance <= threshold) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
    left_distance = analogRead(left_ultra);
    right_distance = analogRead(right_ultra);
    distance_error = left_distance - right_distance;
    Serial.print("error");
    Serial.println(heading_error);

    // PID control for heading correction
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);
    Serial.print("adjustment");
    Serial.println(heading_adjustment);
    // PID control for centering adjustment
    //float centering_adjustment = 0;
    float centering_adjustment = pidController(distance_error, integral_center, prev_error_center, kp_center, ki_center, kd_center, deltaT);

    // Adjust motor speeds
    int left_motor_speed = base_speed - heading_adjustment - centering_adjustment;
    int right_motor_speed = base_speed + heading_adjustment + centering_adjustment;

    // Clamp motor speeds between 0 and 255
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Apply speeds to motors
    analogWrite(ENB, left_motor_speed);
    analogWrite(ENA, right_motor_speed);
  }

  // Ensure both motors stop
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}

void reset_to_initial_heading(int base_speed) {
  float heading_error;
  while (true) {
    updateYaw();
    current_heading = getGyroHeading();
    heading_error = target_heading - current_heading;
    Serial.println(heading_error);
    if (heading_error > 1.0) {
      motor_right(base_speed);
      motor_left(-base_speed);
      Serial.println("turning left");
    } else if (heading_error < -1.0) {
      motor_right(-base_speed);
      motor_left(base_speed);
      Serial.println("turning right");
    }
    if (abs(heading_error) < 1.0) {
      motor_right(0);
      motor_left(0);
      break;
    }
  }

  // Ensure both motors stop after alignment
  Serial.println("stop");
  motor_right(0);
  motor_left(0);
}

void turn_right(int base_speed, float turn_angle) {
  // First, reset the robot to the initial heading
  reset_to_initial_heading(base_speed);

  // Set the target heading to the initial heading + desired turn angle
  float initial_heading = getGyroHeading();
  target_heading = initial_heading + turn_angle;

  unsigned long previousTime = millis();

  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;

    // Get the current heading
    current_heading = getGyroHeading();

    // Calculate the error between the target heading and current heading
    float heading_error = target_heading - current_heading;

    // If the error is close to zero (within a threshold), stop the turn
    if (abs(heading_error) < 1.0) {
      analogWrite(ENB, 0);  // Stop left motor
      analogWrite(ENA, 0);  // Stop right motor
      break;
    }

    // Use PID to compute the adjustment for turning
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);

    // Adjust motor speeds for right turn
    int left_motor_speed = base_speed + heading_adjustment;
    int right_motor_speed = base_speed - heading_adjustment;

    // Clamp motor speeds between 0 and 255
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Apply speeds to motors (left motor forward, right motor backward)
    analogWrite(ENB, left_motor_speed);
    analogWrite(ENA, right_motor_speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);  // Right motor backward
    digitalWrite(IN2, HIGH);
  }

  // Ensure both motors stop after the turn
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}

void turn_left(int base_speed, float turn_angle) {
  // First, reset the robot to the initial heading
  reset_to_initial_heading(base_speed);

  // Set the target heading to the initial heading - desired turn angle
  float initial_heading = getGyroHeading();
  target_heading = initial_heading - turn_angle;

  unsigned long previousTime = millis();

  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;

    // Get the current heading
    current_heading = getGyroHeading();

    // Calculate the error between the target heading and current heading
    float heading_error = target_heading - current_heading;

    // If the error is close to zero (within a threshold), stop the turn
    if (abs(heading_error) < 1.0) {
      analogWrite(ENB, 0);  // Stop left motor
      analogWrite(ENA, 0);  // Stop right motor
      break;
    }

    // Use PID to compute the adjustment for turning
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);

    // Adjust motor speeds for left turn
    int left_motor_speed = base_speed - heading_adjustment;
    int right_motor_speed = base_speed + heading_adjustment;

    // Clamp motor speeds between 0 and 255
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Apply speeds to motors (left motor backward, right motor forward)
    analogWrite(ENB, left_motor_speed);
    analogWrite(ENA, right_motor_speed);
    digitalWrite(IN3, LOW);  // Left motor backward
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, HIGH);  // Right motor forward
    digitalWrite(IN2, LOW);
  }

  // Ensure both motors stop after the turn
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}
