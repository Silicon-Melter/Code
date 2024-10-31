#include <util/atomic.h>


signed int motor1Speed;
signed int motor2Speed;
unsigned int rpm;     // RPM reading
volatile byte pulses1; // number of pulses for encoder 1
volatile byte pulses2; // number of pulses for encoder 2
unsigned long timeold;
unsigned int pulsesperturnleft = 53;
unsigned int pulsesperturnright = 53;
float wheel_diameter = 67; // Diameter of the wheel in mm 
float wheel_circum = (67/2)*6.28; // Circumference of the wheel
// Right Motor
int encoder2_pin = 2; // Encoder 2 pin actual 2
const int IN2 = 10;  // Motor 2 direction pin 2
const int IN1 = 11; // Motor 2 direction pin 1
const int ENA = 5;  // Motor 2 speed pin

// Left Motor
int encoder1_pin = 3; // Encoder 1 pin default 1
const int ENB = 6;  // Motor 1 speed pin
const int IN4 = 12;  // Motor 1 direction pin 2 actual 7
const int IN3 = 13;  // Motor 1 direction pin 1

#define left_ultra A0
#define right_ultra A2

float left_distance = 0;
float right_distance = 0;
float distance_error = 0;

// Function to handle encoder 1 (left motor) pulse count
void counter1() {
    pulses1++;  // Increment the pulse count for left motor
}

// Function to handle encoder 2 (right motor) pulse count
void counter2() {
    pulses2++;  // Increment the pulse count for right motor
}

void forward_drive(float left_distance, float right_distance, int base_speed) {
    unsigned int left_pulses_needed = (left_distance / wheel_circum) * pulsesperturnleft;
    unsigned int right_pulses_needed = (right_distance / wheel_circum) * pulsesperturnright;

    pulses1 = 0;
    pulses2 = 0;

    float eprev_left = 0;
    float eintegral_left = 0;

    float eprev_right = 0;
    float eintegral_right = 0;

    float kp = 1.0;  // Proportional gain
    float ki = 0.0;  // Integral gain
    float kd = 0.025;  // Derivative gain

    unsigned long previousTime = millis();

    // **Start motors** before the loop
    analogWrite(ENB, base_speed);  // Set left motor speed
    analogWrite(ENA, base_speed);  // Set right motor speed
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);

    // Enter the loop to drive the motors based on encoder feedback
    while (true) {
        unsigned long currentTime = millis();
        float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
        previousTime = currentTime;

        int pulses_left = 0;
        int pulses_right = 0;

        // **Safely read pulses** using ATOMIC_BLOCK
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            pulses_left = pulses1;
            pulses_right = pulses2;
        }

        // Break the loop if both motors have reached the target pulses
        if (pulses_left >= left_pulses_needed && pulses_right >= right_pulses_needed) {
            break;
        }

        // Calculate error for left and right motors
        int error_left = left_pulses_needed - pulses_left;
        int error_right = right_pulses_needed - pulses_right;

        // PID for left motor
        float dedt_left = (error_left - eprev_left) / deltaT;
        eintegral_left += error_left * deltaT;
        float control_left = kp * error_left + ki * eintegral_left + kd * dedt_left;

        // PID for right motor
        float dedt_right = (error_right - eprev_right) / deltaT;
        eintegral_right += error_right * deltaT;
        float control_right = kp * error_right + ki * eintegral_right + kd * dedt_right;

        // Store previous errors
        eprev_left = error_left;
        eprev_right = error_right;

        // Adjust motor speeds
        int left_motor_speed = base_speed + control_left;
        int right_motor_speed = base_speed + control_right;

        // Clamp speeds between 0 and 255
        left_motor_speed = constrain(left_motor_speed, 155, 255);
        right_motor_speed = constrain(right_motor_speed, 155, 255);

        // Update motor speeds
        analogWrite(ENB, left_motor_speed);
        analogWrite(ENA, right_motor_speed);

        // Stop motors if they reach the required pulses 
        if (pulses_left >= left_pulses_needed) {
            analogWrite(ENB, 0);  // Stop left motor
        }
        if (pulses_right >= right_pulses_needed) {
            analogWrite(ENA, 0);  // Stop right motor
        }
    }

    // Ensure both motors stop after the loop
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

double computeDistancePID(double error) {
    float Kp = 2.35; 
    float Ki = 0.0; 
    float Kd = 0.0; 
    float integral,previousError;
    unsigned long currentTime = millis();
    static unsigned long previousTime = 0;
    double deltaT = (currentTime - previousTime) / 1000.0;  // Convert to seconds
    previousTime = currentTime;

    double P = Kp * error;

    integral += error * deltaT;
    double I = Ki * integral;

    double derivative = (error - previousError) / deltaT;
    double D = Kd * derivative;

    previousError = error;

    return P + I + D;
}

void forward_drive_with_ultra(float distance, int base_speed) {
    unsigned int left_pulses_needed = (distance / wheel_circum) * pulsesperturnleft;
    unsigned int right_pulses_needed = (distance / wheel_circum) * pulsesperturnright;

    pulses1 = 0;
    pulses2 = 0;

    unsigned long previousTime = millis();

    // Start motors initially
    analogWrite(ENB, base_speed);
    analogWrite(ENA, base_speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);

    while (true) {
        unsigned long currentTime = millis();
        float deltaT = (float)(currentTime - previousTime) / 1000.0;  // in seconds
        previousTime = currentTime;

        int pulses_left, pulses_right;

        // Atomically read encoder pulses
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            pulses_left = pulses1;
            pulses_right = pulses2;
        }

        // Break the loop if both motors reach target pulses
        if (pulses_left >= left_pulses_needed && pulses_right >= right_pulses_needed) {
            break;
        }

        // Ultrasonic distance readings
        float left_distance = analogRead(left_ultra);  // Adjust with any necessary conversion
        float right_distance = analogRead(right_ultra);

        // Calculate distance error between left and right sides
        float distance_error = right_distance - left_distance;

        // Compute PID output to adjust motor speed based on distance error
        float output = computeDistancePID(distance_error);

        // Adjust motor speeds using base speed and PID output
        int leftMotorSpeed = base_speed + output;
        int rightMotorSpeed = base_speed - output;

        // Clamp speeds within allowed limits
        leftMotorSpeed = constrain(leftMotorSpeed, 155, 255);
        rightMotorSpeed = constrain(rightMotorSpeed, 155, 255);

        // Apply adjusted speeds to motors
        analogWrite(ENB, leftMotorSpeed);
        analogWrite(ENA, rightMotorSpeed);

        // Stop motors if either motor reaches required pulses independently
        if (pulses_left >= left_pulses_needed) {
            analogWrite(ENB, 0);  // Stop left motor
        }
        if (pulses_right >= right_pulses_needed) {
            analogWrite(ENA, 0);  // Stop right motor
        }
    }

    // Ensure both motors are stopped after loop
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void turn_left(int speed, int time){
    analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);  // Left motor forward
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void turn_right(int speed, int time){
    analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, HIGH);   // Right motor forward
    digitalWrite(IN2, LOW);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void uturn_left(int speed, int time, int distance){
  analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);  // Left motor forward
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
    delay(300);
    forward_drive(distance,distance,speed),
    delay(300);
    analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);  // Left motor forward
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void uturn_right(int speed, int time, int distance){
  analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, HIGH);   // Right motor forward
    digitalWrite(IN2, LOW);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
    delay(300);
    forward_drive(distance,distance,speed),
    delay(300);
    analogWrite(ENB, speed);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, HIGH);   // Right motor forward
    digitalWrite(IN2, LOW);
    delay(time);
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void forward_drive_with_bang_ultra(float left_distance, float right_distance, int base_speed) {
    unsigned int left_pulses_needed = (left_distance / wheel_circum) * pulsesperturnleft;
    unsigned int right_pulses_needed = (right_distance / wheel_circum) * pulsesperturnright;

    pulses1 = 0;
    pulses2 = 0;

    // Start motors with initial speed
    analogWrite(ENB, base_speed);
    analogWrite(ENA, base_speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);   // Right motor forward
    digitalWrite(IN2, HIGH);

    while (true) {
        int pulses_left, pulses_right;

        // Atomically read encoder pulses
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            pulses_left = pulses1;
            pulses_right = pulses2;
        }

        // Break the loop if both motors reach the target pulses
        if (pulses_left >= left_pulses_needed && pulses_right >= right_pulses_needed) {
            break;
        }

        // Read ultrasonic distances
        float left_ultra_distance = analogRead(left_ultra);  // Adjust with any necessary conversion
        float right_ultra_distance = analogRead(right_ultra);

        Serial.print("Left:");
        Serial.println(left_ultra_distance);
        Serial.print("Right:");
        Serial.println(right_ultra_distance);

        // Calculate the distance error
        float distance_error = left_ultra_distance - right_ultra_distance;

        // Bang-bang control logic based on distance error
        if (distance_error < 0) {
            // Robot is closer to the left wall, speed up the left motor
            analogWrite(ENB, 215);
            analogWrite(ENA, 155);
        } else if (distance_error > 0) {
            // Robot is closer to the right wall, speed up the right motor
            analogWrite(ENA, 185);
            analogWrite(ENB, 155);
        } else {
            // If centered, maintain base speed for both motors
            analogWrite(ENB, base_speed);
            analogWrite(ENA, base_speed);
        }

        // Stop motors if either motor reaches the required pulses independently
        if (pulses_left >= left_pulses_needed) {
            analogWrite(ENB, 0);  // Stop left motor
        }
        if (pulses_right >= right_pulses_needed) {
            analogWrite(ENA, 0);  // Stop right motor
        }
    }

    // Ensure both motors stop after the loop
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}


void drive_left_motor(float left_distance, int left_speed) {
    // Calculate the number of pulses required to move the desired distance for the left motor
    unsigned int left_pulses_needed = (left_distance / wheel_circum) * pulsesperturnleft;

    // Reset pulse counter for the left motor
    pulses1 = 0;

    // Set motor speed and direction to move forward
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    analogWrite(ENB, left_speed);

    // Drive the left motor until the desired pulse count (distance) is reached
    while (pulses1 < left_pulses_needed) {
        
        // Detach interrupts to safely check the pulse count
        detachInterrupt(digitalPinToInterrupt(encoder1_pin));

        if (pulses1 >= left_pulses_needed) {
            analogWrite(ENB, 0);  // Stop left motor
        }
        Serial.print("still in loop");
        Serial.println(pulses1);
        // Re-attach interrupt to continue pulse counting
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
    }

    // Ensure the left motor is stopped
    analogWrite(ENB, 0);
    Serial.print("break++++++++++++++");
    Serial.println(pulses1);
}

void drive_right_motor(float right_distance, int right_speed) {
    // Calculate the number of pulses required to move the desired distance for the right motor
    unsigned int right_pulses_needed = (right_distance / wheel_circum) * pulsesperturnright;

    // Reset pulse counter for the right motor
    pulses2 = 0;

    // Set motor speed and direction to move forward
    digitalWrite(IN1, LOW);  // Right motor forward
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, right_speed);

    // Drive the right motor until the desired pulse count (distance) is reached
    while (pulses2 < right_pulses_needed) {
        
        // Detach interrupts to safely check the pulse count
        detachInterrupt(digitalPinToInterrupt(encoder2_pin));

        if (pulses2 >= right_pulses_needed) {
            analogWrite(ENA, 0);  // Stop right motor
        }
        Serial.print("still in loop");
        Serial.println(pulses2);
        // Re-attach interrupt to continue pulse counting
        attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
    }

    // Ensure the right motor is stopped
    analogWrite(ENA, 0);
    Serial.print("break");
    Serial.println(pulses2);
}

void motor(int right_speed,int left_speed) {
    digitalWrite(IN1, LOW);  // Right motor forward
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, right_speed);
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    analogWrite(ENB, left_speed);
}

void setup() {
    Serial.begin(9600);

    pinMode(encoder1_pin, INPUT);
    pinMode(encoder2_pin, INPUT);
    
    // Right Motor setup
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Left Motor setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(left_ultra, INPUT);
    pinMode(right_ultra, INPUT);

    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);

    pulses1 = 0;
    pulses2 = 0;
    timeold = millis();

    delay(3000);
    forward_drive_with_ultra(330,155);
    delay(300);
    turn_right(155,500);
    forward_drive(300,300,155);
    delay(300);
    forward_drive_with_ultra(560,155);
    delay(300);
    turn_right(155,500);
    forward_drive(100,100,155);
    delay(300);
    forward_drive_with_ultra(330,155);
    delay(300);
    uturn_left(155,500,150);
}

void loop() {
  // delay(3000);
    //motor(255,255);
    // Drive forward for 600 mm on both motors with 100 speed on each motor
    //forward_drive(800, 800, 155);
    //drive_right_motor(210,100);
    //drive_left_motor(210,100);
    // forward_drive_with_ultra(370,155);
    // delay(300);
    // turn_right(155,500);
    //forward_drive_with_bang_ultra(800,800,155);
    //forward_drive(630, 630, 155);
    // delay(3000);
}
