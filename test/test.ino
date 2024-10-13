signed int motorSpeed;          // Changed to single motor speed variable for simplification
unsigned int rpm;               // RPM reading
volatile unsigned long pulses1; // number of pulses for encoder 1
volatile unsigned long pulses2; // number of pulses for encoder 2
unsigned long timeold;
int encoder1_pin = 2; // Encoder 1 pin (if needed later)
int encoder2_pin = 3; // Encoder 2 pin
unsigned int pulsesperturn = 20;
float wheel_diameter = 67;                            // Diameter of the wheel
float wheel_circum = 2 * 3.14 * (wheel_diameter / 2); // Circumference of the wheel

const int IN1 = 7;
const int IN2 = 8;
const int ENA = 5;
const int num_iterations = 10; // Number of iterations for averaging

unsigned long pulse_counts[num_iterations]; // Array to store pulse counts
int sum = 0;
float avg = 0;
float uncertainty = 0; // Variable to store uncertainty

void counter1()
{
    pulses1++; // Increment pulse count for encoder 1 (if used)
}

void counter2()
{
    pulses2++; // Increment pulse count for encoder 2
}

void setup()
{
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(encoder1_pin, INPUT);                                            // Ensure encoder 1 pin is set
    pinMode(encoder2_pin, INPUT);                                            // Set encoder 2 pin
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING); // Attach interrupt for encoder 1
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING); // Attach interrupt for encoder 2

    pulses1 = 0; // Initialize pulse counts
    pulses2 = 0;
    timeold = millis(); // Initialize timer
}

void loop()
{
    for (int i = 0; i < num_iterations; i++) // Loop for the specified number of iterations
    {
        digitalWrite(IN1, HIGH); // Set motor direction
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 155); // Set motor speed

        if (millis() - timeold >= 1000)
        {
            // Disable interrupts for calculation
            detachInterrupt(digitalPinToInterrupt(encoder1_pin));
            detachInterrupt(digitalPinToInterrupt(encoder2_pin));

            // Calculate distance for both encoders
            float distance2 = (wheel_circum / pulsesperturn) * pulses2; // Distance calculation for encoder 2
            Serial.print("Encoder 2 Pulse = ");
            Serial.println(pulses2, DEC);
            Serial.print("Encoder 2 Distance = ");
            Serial.println(distance2, 2); // Display distance2 with 2 decimal places
            
            // Store the pulses2 value in the array
            pulse_counts[i] = pulses2;

            // Reset pulse counts and time
            pulses2 = 0;        // Reset encoder 2 pulses
            timeold = millis(); // Reset timer

            // Re-enable interrupts
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
            attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
        }

        delay(1000); // Delay to allow for next iteration
        // Stop the motor after the delay
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0); // Stop the motor
        delay(1000);
    }

    // Calculate average
    for (int i = 0; i < num_iterations; i++) {
        sum += pulse_counts[i];
    }
    avg = sum / (float)num_iterations;

    // Calculate uncertainty (standard error of the mean)
    float sum_squared_diff = 0.0;
    for (int i = 0; i < num_iterations; i++) {
        sum_squared_diff += pow(pulse_counts[i] - avg, 2);
    }
    float variance = sum_squared_diff / num_iterations;
    uncertainty = sqrt(variance) / sqrt(num_iterations); // Standard deviation divided by sqrt(n)

    // Print results
    Serial.print("Average Pulses: ");
    Serial.println(avg);
    Serial.print("Uncertainty: ");
    Serial.println(uncertainty);

    // Reset for next loop
    sum = 0;
    for (int i = 0; i < num_iterations; i++) {
        pulse_counts[i] = 0; // Reset the pulse counts array
    }

    // Wait before the next iteration
    delay(5000); // Delay before restarting the loop
}
