#include <TimerOne.h>
#include <Ultrasonic.h>
#include <Servo.h>

// Define pins
const int trigPin = 49; 
const int echoPin = 47;
const int grabberPin = 51; 
Servo grabber;

// Ultrasonic pins
Ultrasonic ultrasonic(trigPin, echoPin);

// Constants for calculations
#define PI 3.1415926535897932384626433832795
float d = 0.172; // Wheel distance
float r = 0.0225; // Wheel radius
float samplingTime = 0.2; // Sampling time
const int ENCODER_RESOLUTION = 700;
float M_PER_REV = 2 * PI * r;

// Motor control pins
int enR = 7;
int inR1 = 9;
int inR2 = 8;
int enL = 5;
int inL1 = 10;
int inL2 = 11;

// Encoder pins
int enLA = 2;
int enLB = 3;
int enRA = 20;
int enRB = 21;

// Encoder counts
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// PID constants
float KpL = 100;  
float KiL = 260;  
float KdL = 33;  
float KpR = 100;  
float KiR = 130;  
float KdR = 33;  

float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0;

// Add a flag to indicate if the grabber is closed
bool isGrabberClosed = false;

void setup() {
    Serial.begin(9600); 
    grabber.attach(grabberPin);
    delay(500);
   grabber.write(0);
   delay(500); 

    // Setup interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
    attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
    attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
    attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

    // Set motor control pins to outputs
    pinMode(enR, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(inR1, OUTPUT);
    pinMode(inR2, OUTPUT);
    pinMode(inL1, OUTPUT);
    pinMode(inL2, OUTPUT);
    
    // Turn off motors initially
    stop();

    // Initialize TimerOne for velocity control
    Timer1.initialize(1000000 * samplingTime); // 1,000,000 microseconds = 1 second
    Timer1.attachInterrupt(VelCtrlTimer); // Attach the interrupt to the velocity control function
}

void VelCtrlTimer() {
    // Calculate wheel velocities in m/s
    float vR = (float(rightEnCount) / ENCODER_RESOLUTION) * M_PER_REV / samplingTime;
    float vL = (float(leftEnCount) / ENCODER_RESOLUTION) * M_PER_REV / samplingTime;

    // Reset encoder counts for the next calculation
    leftEnCount = 0;
    rightEnCount = 0;

    // PID calculations for left and right motor speeds
    err_vL = set_vL - vL;
    err_vR = set_vR - vR;
  
    integralL += err_vL * samplingTime;
    integralR += err_vR * samplingTime;
    float controlOutputL = KpL * err_vL + KiL * integralL + KdL * (err_vL - pre_err_vL) / samplingTime;
    float controlOutputR = KpR * err_vR + KiR * integralR + KdR * (err_vR - pre_err_vR) / samplingTime;

    pre_err_vL = err_vL;
    pre_err_vR = err_vR;

    // Set motor speeds
    setMotorSpeedL((int)controlOutputL);
    setMotorSpeedR((int)controlOutputR);
}

void setMotorSpeedL(int speed) {
    // Set motor direction and speed for left motor
    if (speed == 0) {
        digitalWrite(inL1, LOW);
        digitalWrite(inL2, LOW);
    } else if (speed > 0) {
        digitalWrite(inL1, LOW);
        digitalWrite(inL2, HIGH);
    } else {
        digitalWrite(inL1, HIGH);
        digitalWrite(inL2, LOW);
    }

    // Set PWM speed
    speed = constrain(abs(speed), 30, 255); // Ensure speed is within range
    analogWrite(enL, speed);
}

void setMotorSpeedR(int speed) {
    // Set motor direction and speed for right motor
    if (speed == 0) {
        digitalWrite(inR1, LOW);
        digitalWrite(inR2, LOW);
    } else if (speed > 0) {
        digitalWrite(inR1, LOW);
        digitalWrite(inR2, HIGH);
    } else {
        digitalWrite(inR1, HIGH);
        digitalWrite(inR2, LOW);
    }

    // Set PWM speed
    speed = constrain(abs(speed), 30, 255); // Ensure speed is within range
    analogWrite(enR, speed);
}

void stop() {
    // Turn off motors 
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
}

// Encoder ISR functions
void leftEnISRA() {
    leftEnCount += (digitalRead(enLB) == HIGH) ? -1 : 1;
}

void leftEnISRB() {
    leftEnCount += (digitalRead(enLA) == HIGH) ? 1 : -1;
}

void rightEnISRA() {
    rightEnCount += (digitalRead(enRB) == HIGH) ? -1 : 1;
}

void rightEnISRB() {
    rightEnCount += (digitalRead(enRA) == HIGH) ? 1 : -1;
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');  // Read until newline
        Serial.print("Raw data: '"); Serial.print(data); Serial.println("'");  // Debug output
        data.trim();  // Remove whitespace

        int separatorIndex = data.indexOf(' ');  // Find space between velocities
        if (separatorIndex != -1) {
            // Split the data into set_vL and set_vR
            set_vL = data.substring(0, separatorIndex).toFloat();
            set_vR = data.substring(separatorIndex + 1).toFloat();
        }
    }

    // Measure distance from ultrasonic sensor
    float distance = ultrasonic.read();

 if (distance <= 3.5) {
        delay(1000);
        grabber.write(180); // Close grabber
        isGrabberClosed = true; // Set flag to true
        delay(1000);
        Serial.print("Distance: ");
        Serial.println(distance);
        stop(); // Stop robot
        set_vL = 0;
        set_vR = 0;
        // Print message when grabber is closed
        if (isGrabberClosed) {
            Serial.println("Grabber closed, robot stopped.");
        }
        while(1){};
    }
}
