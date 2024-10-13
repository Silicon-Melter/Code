signed int motor1Speed;
signed int motor2Speed;
unsigned int rpm;     // rpm reading
volatile byte pulses1; // number of pulses for encoder 1
volatile byte pulses2; // number of pulses for encoder 2
unsigned long timeold;
int encoder1_pin = 2; // Encoder 1 pin
int encoder2_pin = 3; // Encoder 2 pin
unsigned int pulsesperturn = 20;
float wheel_diameter = 67; // Diameter of the wheel
float wheel_circum = 2 * 3.14 * (wheel_diameter / 2); // Circumference of the wheel

const int IN1 = 7;
const int IN2 = 8;
const int ENA = 5;
void counter1()
{
    pulses1++;
}

void counter2()
{
    pulses2++;
}

void cal_distance(float *distance1, float *distance2)
{
    if (millis() - timeold >= 1000)
    {
        // Disable interrupts for calculation
        detachInterrupt(digitalPinToInterrupt(encoder1_pin));
        detachInterrupt(digitalPinToInterrupt(encoder2_pin));

        // Calculate distance for both encoders
        *distance1 = (wheel_circum / pulsesperturn) * pulses1;
        *distance2 = (wheel_circum / pulsesperturn) * pulses2;

        Serial.print("Encoder 1 Pulse = ");
        Serial.println(pulses1, DEC);
        Serial.print("Encoder 1 Distance = ");
        Serial.println(*distance1, 2); // Display distance1 with 2 decimal places

        Serial.print("Encoder 2 Pulse = ");
        Serial.println(pulses2, DEC);
        Serial.print("Encoder 2 Distance = ");
        Serial.println(*distance2, 2); // Display distance2 with 2 decimal places

        // Reset pulse counts and time
        pulses1 = 0;
        pulses2 = 0;
        timeold = millis();

        // Re-enable interrupts
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
        attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
    }
}

void forward_drive(float desired_distance1, float desired_distance2, int motorspeed1, int motorspeed2)
{
    float actual_distance1 = 0;
    float actual_distance2 = 0;

    // Set motor speed to start moving
    analogWrite(motor1Speed, motorspeed1);
    analogWrite(motor2Speed, motorspeed2);

    // Loop until both motors reach their desired distance
    while (actual_distance1 < desired_distance1 || actual_distance2 < desired_distance2)
    {
        // Update distances for both wheels
        cal_distance(&actual_distance1, &actual_distance2);

        // Stop motor 1 if it reaches its desired distance
        if (actual_distance1 >= desired_distance1)
        {
            analogWrite(motor1Speed, 0);
            Serial.println("Motor 1 Stopped");
        }

        // Stop motor 2 if it reaches its desired distance
        if (actual_distance2 >= desired_distance2)
        {
            analogWrite(motor2Speed, 0);
            Serial.println("Motor 2 Stopped");
        }
    }

    // Ensure both motors are stopped
    analogWrite(motor1Speed, 0);
    analogWrite(motor2Speed, 0);

    // Reset pulse counts for both encoders
    pulses1 = 0;
    pulses2 = 0;
}
void motor(int L_sp,int R_sp){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,L_sp);
  delay(R_sp);
}

void setup()
{
    Serial.begin(9600);

    pinMode(encoder1_pin, INPUT);
    pinMode(encoder2_pin, INPUT);
    pinMode(motor1Speed, OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(ENA,OUTPUT);
    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);

    pulses1 = 0;
    pulses2 = 0;
    timeold = millis();
}

void loop()
{
  
  if (millis() - timeold >= 1000)
    {
        // Disable interrupts for calculation
        detachInterrupt(digitalPinToInterrupt(encoder1_pin));
        detachInterrupt(digitalPinToInterrupt(encoder2_pin));

        // Calculate distance for both encoders
        float distance2 = (wheel_circum / pulsesperturn) * pulses2;
        Serial.print("Encoder 2 Pulse = ");
        Serial.println(pulses2, DEC);
        Serial.print("Encoder 2 Distance = ");
        Serial.println(distance2, 2); // Display distance2 with 2 decimal places

        // Reset pulse counts and time
        
        timeold = millis();

        // Re-enable interrupts
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), counter1, FALLING);
        attachInterrupt(digitalPinToInterrupt(encoder2_pin), counter2, FALLING);
    }
  delay(5000);
  motor(100,1000);
  motor(0,5000);
  
}
