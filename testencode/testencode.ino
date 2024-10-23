signed int motor1Speed;
signed int motor2Speed;
unsigned int rpm;     // RPM reading
volatile byte pulses1; // number of pulses for encoder 1
volatile byte pulses2; // number of pulses for encoder 2
unsigned long timeold;
unsigned int pulsesperturnleft = 44;
unsigned int pulsesperturnright = 33;
float wheel_diameter = 67; // Diameter of the wheel in mm 
float wheel_circum = (67/2)*6.28; // Circumference of the wheel
// Right Motor
int encoder2_pin = 3; // Encoder 2 pin actual 2
const int IN2 = 9;  // Motor 2 direction pin 2
const int IN1 = 10; // Motor 2 direction pin 1
const int ENA = 11;  // Motor 2 speed pin

// Left Motor
int encoder1_pin = 2; // Encoder 1 pin default 1
const int ENB = 6;  // Motor 1 speed pin
const int IN4 = 7;  // Motor 1 direction pin 2 actual 7
const int IN3 = 8;  // Motor 1 direction pin 1

// Function to handle encoder 1 (left motor) pulse count
void counter1() {
    pulses1++;  // Increment the pulse count for left motor
}

// Function to handle encoder 2 (right motor) pulse count
void counter2() {
    pulses2++;  // Increment the pulse count for right motor
}


void forward_drive(float left_distance, float right_distance, int left_speed, int right_speed) {
    // Calculate the number of pulses required to move the desired distance for each motor
    unsigned int left_pulses_needed = (left_distance / wheel_circum) * pulsesperturnleft;
    unsigned int right_pulses_needed = (right_distance / wheel_circum) * pulsesperturnright;

    // Reset pulse counters
    pulses1 = 0;
    pulses2 = 0;

    // Set motor speeds and directions to move forward
    digitalWrite(IN3, HIGH);  // Left motor forward
    digitalWrite(IN4, LOW);
    analogWrite(ENB, left_speed);

    digitalWrite(IN1, LOW);  // Right motor forward
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, right_speed);

    // Drive motors until the desired pulse count (distance) is reached
    while (pulses1 < left_pulses_needed || pulses2 < right_pulses_needed) {
        
        // Detach interrupts to prevent them from firing while checking pulse counts
        detachInterrupt(digitalPinToInterrupt(encoder1_pin));
        detachInterrupt(digitalPinToInterrupt(encoder2_pin));

        if (pulses1 >= left_pulses_needed) {
            analogWrite(ENB, 0);  // Stop left motor
        }
        if (pulses2 >= right_pulses_needed) {
            analogWrite(ENA, 0);  // Stop right motor
        }

        // Re-attach interrupts after checking pulse counts to continue pulse counting
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
        attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
    }

    // Ensure both motors are stopped after reaching the desired distance
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

    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);

    pulses1 = 0;
    pulses2 = 0;
    timeold = millis();
}

void loop() {
    delay(3000);  // Wait for 3 seconds before starting the motor drive
    
    // Drive forward for 67 mm on both motors with 100 speed on each motor
    forward_drive(210, 210, 100, 100);
    //drive_right_motor(210,100);
    //drive_left_motor(210,100);

}
