#include <Servo.h> // Include the Servo library

// Motor Pins
int enL = 6;
int inL1 = 8;
int inL2 = 9;
int enR = 7;
int inR1 = 10;
int inR2 = 11;

// Velocities variables
int v_left = 0;
int v_right = 0;

// Servo pin
int servo_pin = 49;
Servo gripper;

// Ultrasonic sensor
const int echo = 50;
const int trig = 51;

// Distance threshold
const int threshold = 10; // 10 cm
bool search_bottle = true;

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  gripper.attach(servo_pin);
  gripper.write(90);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  
  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the incoming command
    command.trim();  // Remove any extra spaces or newlines

    // Parse the command
    if (parseCommand(command)) {
      // Update motor speeds
      setMotorSpeeds(v_left, v_right);
    }
  }
  
  // Get distance
  int distancefront = measureDistance();
  
  // Debugging
  Serial.print("Front: ");
  Serial.println(distancefront);
  
  if (search_bottle) {
    if (distancefront > 0 && distancefront <= threshold) {
      stop();
      grab_bottle();
      search_bottle = false; // Stop searching
    } else {
      Serial.println("Do not detect, check camera connection");
    }
  }
}

// Function to parse the incoming command and extract motor velocities
bool parseCommand(String command) {
  int commaIndex = command.indexOf(',');  // Find the comma separator
  if (commaIndex == -1) {
    return false;  // Invalid command format
  }

  // Extract left and right velocities from the command
  v_left = command.substring(0, commaIndex).toInt();
  v_right = command.substring(commaIndex + 1).toInt();

  return true;  // Successfully parsed the command
}

// Function to control motor speeds and directions
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Set left motor direction and speed
  if (leftSpeed >= 0) {
    digitalWrite(inL1, HIGH); 
    digitalWrite(inL2, LOW); // Forward
  } else {
    digitalWrite(inL2, HIGH);
    digitalWrite(inL1, LOW);   // Backward
    leftSpeed = -leftSpeed;             // Use absolute value for PWM
  }
  analogWrite(enL, constrain(leftSpeed, 0, 255));

  // Set right motor direction and speed
  if (rightSpeed >= 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);  // Forward
  } else {
    digitalWrite(inR2, HIGH);
    digitalWrite(inR1, LOW);   // Backward
    rightSpeed = -rightSpeed;            // Use absolute value for PWM
  }
  analogWrite(enR, constrain(rightSpeed, 0, 255));
}

void stop() {
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

int measureDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2); // 2 microseconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10); // 10 microseconds
  digitalWrite(trig, LOW);
  
  long duration = pulseIn(echo, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm 
  return distance; 
}

void grab_bottle() {
  Serial.println("Bottle detected. Grabbing...");
  gripper.write(0);  // Close the gripper
  delay(1000);
  gripper.write(90);  // Open the gripper
}
