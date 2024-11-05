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
float kp_heading = 2, ki_heading = 0, kd_heading = 0.1;
float kp_center = 1.5, ki_center = 0.0, kd_center = 0.05;

const int LEFTtrig = 0;  
const int LEFTecho = 1; 
const int FRONTtrig = 4;  
const int FRONTecho = 5; 
const int RIGHTtrig = 12;  
const int RIGHTecho = 13; 

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFTtrig, OUTPUT);  
	pinMode(LEFTecho, INPUT);  
  pinMode(FRONTtrig, OUTPUT);  
	pinMode(FRONTecho, INPUT);  
  pinMode(RIGHTtrig, OUTPUT);  
	pinMode(RIGHTecho, INPUT);  

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
  driveStraightWithPID(150,155);
  //run(1);
}

void loop() {
 
}


void run(int where){
  delayMillis(3000);
  pid_gyro_leftwall(200.0, 135);
  delayMillis(100);
  turn_right(145,85);
  delayMillis(100);
  pid_gyro_leftwall(200.0, 135);
  delayMillis(100);
  turn_right(145,85);
  delayMillis(100);
  pid_gyro(200.0, 135);
  delayMillis(100);
  turn_left(145,85);
  delayMillis(100);
  pid_gyro_rightwall(120.0, 105);
  delayMillis(100);
  turn_left(145,85);
  delayMillis(100);
  pid_gyro_rightwall_tillfound(155);
  delayMillis(100);
  pid_gyro_rightwall_tillgone(155);
  delayMillis(100);
  turn_left(145,85);
  delayMillis(100);
  pid_gyro_tillfound(135,0);
  delayMillis(100);
  pid_gyro_leftwall_tillgone(165);
  delayMillis(100);
  turn_right(145,85);
  pid_gyro_time(0.4,155);
  turn_right(145,85);
  delayMillis(100);
  pid_gyro(150.0, 135);
  delayMillis(100);
  turn_left(145,85);
  delayMillis(100);
  pid_gyro_rightwall_tillfound(155);
  pid_gyro_rightwall_tillgone(155);\
  switch(where){
    case 1:
      go_one();
      break;
    case 2:
      go_two();
      break;
    default:
      break;
  }
}

void go_one(){
  turn_left(145,85);
  delayMillis(100);
  pid_gyro_leftwall(200.0, 135);
  turn_right(145,85);
  delayMillis(100);
  pid_gyro_leftwall(200.0, 135);
  delayMillis(100);
  turn_right(145,85);
  delayMillis(100);
  pid_gyro_leftwall(150.0, 135);
}

void go_two(){
  turn_right(145,85);
  delayMillis(100);
  pid_gyro(150,135);
}

float calculateDistance(int trigPin, int echoPin) {
  // Send a pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  float distance = duration * 0.343 / 2;

  return distance;
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
    yaw += (norm.ZAxis/10) * deltaT;
}

// Modify getGyroHeading to simply return the updated yaw
float getGyroHeading() {
    updateYaw();
    return yaw;
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
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  // Left motor backward
    digitalWrite(IN4, HIGH);
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
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    if (front_distance <= threshold) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    distance_error = left_distance - 70;
    Serial.print("error");
    Serial.println(heading_error);

    // PID control for heading correction
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);
    Serial.print("adjustment");
    Serial.println(heading_adjustment);
    // PID control for centering adjustment
    float centering_adjustment = 0;
    //float centering_adjustment = pidController(distance_error, integral_center, prev_error_center, kp_center, ki_center, kd_center, deltaT);

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

void pid_gyro_leftwall(float threshold, int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    if (front_distance <= threshold) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    distance_error = left_distance - 70;
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

void pid_gyro_leftwall_tillgone(int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    if (right_distance > 150) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    distance_error = left_distance - 70;
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

void pid_gyro_leftwall_tillfound(int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    if (right_distance < 150) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    distance_error = left_distance - 70;
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

void pid_gyro_rightwall(float threshold, int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    if (front_distance <= threshold) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    distance_error = 65 - right_distance;
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

void pid_gyro_rightwall_tillgone(int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    if (left_distance > 150) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
  
    distance_error = 65 - right_distance;
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

void pid_gyro_rightwall_tillfound(int base_speed) {
  
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    if (left_distance < 150) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // Ultrasonic centering
  
    distance_error = 65 - right_distance;
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

void pid_gyro(float threshold, int base_speed) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    if (front_distance <= threshold) {
      analogWrite(ENB, 0);
      analogWrite(ENA, 0);
      break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // PID control for heading correction
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);
    Serial.print("adjustment");
    Serial.println(heading_adjustment);
    // Adjust motor speeds
    int left_motor_speed = base_speed - heading_adjustment;
    int right_motor_speed = base_speed + heading_adjustment;

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
  reset_to_initial_heading(base_speed);
}

void pid_gyro_time(float time, int base_speed) {
  // Capture the start time
  unsigned long startTime = millis();
  target_heading = getGyroHeading();

  unsigned long previousTime = millis();

  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);

  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;

    // Calculate elapsed time
    float elapsedTime = (currentTime - startTime) / 1000.0;  // Convert to seconds

    // Stop if the elapsed time exceeds the specified duration
    if (elapsedTime >= time) {
      analogWrite(ENB, 0);  // Stop left motor
      analogWrite(ENA, 0);  // Stop right motor
      break;
    }

    // Read the front ultrasonic distance (if obstacle check needed)
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // PID control for heading correction
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);

    // Adjust motor speeds
    int left_motor_speed = base_speed - heading_adjustment;
    int right_motor_speed = base_speed + heading_adjustment;

    // Clamp motor speeds between 0 and 255
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Apply speeds to motors
    analogWrite(ENB, left_motor_speed);
    analogWrite(ENA, right_motor_speed);

    // Debugging output for monitoring adjustments
    Serial.print("Heading Adjustment: ");
    Serial.println(heading_adjustment);
  }

  // Ensure both motors stop after time limit
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);

  // Optionally, reset the heading after the motion
  reset_to_initial_heading(base_speed);
}


void pid_gyro_tillfound(int base_speed, int mode) {
  target_heading = getGyroHeading();
  unsigned long previousTime = millis();
  bool condition;
  // Start motors
  motor_left(base_speed);
  motor_right(base_speed);
  
  while (true) {
    unsigned long currentTime = millis();
    float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
    previousTime = currentTime;
    float front_distance = calculateDistance(FRONTtrig,FRONTecho);
    left_distance = calculateDistance(LEFTtrig,LEFTecho);
    right_distance = calculateDistance(RIGHTtrig,RIGHTecho);
    switch(mode) {
        case 1:
            //left
            condition = left_distance < 150;
            break;
        case 2:
            //rights
            condition = right_distance < 150;
            break;
        default:
            //either
            condition = left_distance < 150 || right_distance < 150;
            break;
    }
    if (condition) {
        analogWrite(ENB, 0);
        analogWrite(ENA, 0);
        break;
    }

    // Update current heading and calculate heading error
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;

    // PID control for heading correction
    float heading_adjustment = pidController(heading_error, integral_heading, prev_error_heading, kp_heading, ki_heading, kd_heading, deltaT);
    Serial.print("adjustment");
    Serial.println(heading_adjustment);
    // Adjust motor speeds
    int left_motor_speed = base_speed - heading_adjustment;
    int right_motor_speed = base_speed + heading_adjustment;

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
  reset_to_initial_heading(base_speed);
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
  target_heading = initial_heading - turn_angle;

  unsigned long previousTime = millis();
  Serial.print("target_heading");
  Serial.println(target_heading);
  while (true) {
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;
    motor_right(-base_speed);
    motor_left(base_speed);
    Serial.println(heading_error);
    // If the error is close to zero (within a threshold), stop the turn
    if (abs(heading_error) < 1.0) {
      analogWrite(ENB, 0);  // Stop left motor
      analogWrite(ENA, 0);  // Stop right motor
      break;
    }

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
  target_heading = initial_heading + turn_angle;

  unsigned long previousTime = millis();

  while (true) {
    current_heading = getGyroHeading();
    float heading_error = target_heading - current_heading;
    motor_right(base_speed);
    motor_left(-base_speed);
    if (abs(heading_error) < 1.0) {
      analogWrite(ENB, 0);  // Stop left motor
      analogWrite(ENA, 0);  // Stop right motor
      break;
    }
  }

  // Ensure both motors stop after the turn
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}
