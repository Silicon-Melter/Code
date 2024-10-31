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
float wheel_circum = (67 / 2) * 6.28; // Circumference of the wheel

// Motor pins
int encoder2_pin = 2; 
const int IN2 = 10;
const int IN1 = 11;
const int ENA = 5;

int encoder1_pin = 3;
const int ENB = 6;
const int IN4 = 12;
const int IN3 = 13;

#define left_ultra A0
#define right_ultra A2

float left_distance = 0;
float right_distance = 0;
float distance_error = 0;

// Kalman filter variables
float estimated_position = 0.0;
float estimated_velocity = 0.0;
float position_variance = 1.0;
float measurement_variance = 0.5;
float process_variance = 1.0;

// Encoder counters
void counter1() { pulses1++; }
void counter2() { pulses2++; }

float kalmanFilter(float measurement, float control_input) {
    // Predict step
    estimated_position += control_input;
    position_variance += process_variance;

    // Measurement update
    float kalman_gain = position_variance / (position_variance + measurement_variance);
    estimated_position += kalman_gain * (measurement - estimated_position);
    position_variance *= (1 - kalman_gain);

    return estimated_position;
}

void forward_drive_with_kalman_wall_following(float distance, int base_speed) {
    unsigned int left_pulses_needed = (distance / wheel_circum) * pulsesperturnleft;
    unsigned int right_pulses_needed = (distance / wheel_circum) * pulsesperturnright;

    pulses1 = 0;
    pulses2 = 0;

    unsigned long previousTime = millis();

    // Start motors
    analogWrite(ENB, base_speed);
    analogWrite(ENA, base_speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    while (true) {
        unsigned long currentTime = millis();
        float deltaT = (float)(currentTime - previousTime) / 1000.0; // in seconds
        previousTime = currentTime;

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

        // Wall-following with ultrasonic sensors
        left_distance = analogRead(left_ultra); // Adjust with necessary conversion
        right_distance = analogRead(right_ultra);
        distance_error = left_distance - right_distance;

        // Apply wall-following logic
        if (distance_error < -20) { // Too close to the left wall
            analogWrite(ENB, base_speed + 20);
            analogWrite(ENA, base_speed - 20);
        } else if (distance_error > 20) { // Too close to the right wall
            analogWrite(ENA, base_speed + 20);
            analogWrite(ENB, base_speed - 20);
        } else {
            analogWrite(ENB, base_speed);
            analogWrite(ENA, base_speed);
        }

        // Kalman filter position estimation
        float encoder_estimate = (pulses_left + pulses_right) / 2.0 * (wheel_circum / pulsesperturnleft);
        float sensor_estimate = (left_distance + right_distance) / 2.0;
        float estimated_position = kalmanFilter(sensor_estimate, encoder_estimate);

        Serial.print("Estimated Position: ");
        Serial.println(estimated_position);

        // Stop motors if either motor reaches required pulses independently
        if (pulses_left >= left_pulses_needed) {
            analogWrite(ENB, 0);
        }
        if (pulses_right >= right_pulses_needed) {
            analogWrite(ENA, 0);
        }
    }

    // Ensure both motors stop after loop
    analogWrite(ENB, 0);
    analogWrite(ENA, 0);
}

void setup() {
    Serial.begin(9600);

    pinMode(encoder1_pin, INPUT);
    pinMode(encoder2_pin, INPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(left_ultra, INPUT);
    pinMode(right_ultra, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);

    pulses1 = 0;
    pulses2 = 0;
    timeold = millis();
}

void loop() {
    delay(3000);  // Wait for 3 seconds before starting

    forward_drive_with_kalman_wall_following(600.0, 155); // Drive 600 mm with wall-following and Kalman

    delay(3000);  // Wait before repeating or executing another command
}
