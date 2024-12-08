#include <Servo.h>

Servo grabberservo; 
const int servopin = 51; 
const int echo = 50;
const int trig = 49;

void setup() {
  grabberServo.attach(servopin); 
  Serial.begin(9600);
  pinMode(trig,OUTPUT);
  pinMODE(echo,INPUT);
}
void ultra() {
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long duration = pulseIn(echo,HIGH);
  long distance = duration * 0.034 / 2; //speed sound 343m/s
}
void loop() {
  long distance = ultra();
  if (distance > 0 && distance <= 10){
    Serial.println("detect");
    grabberservo.write(90);
    delay(3000); 
    Serial.println("Closing Grabber...");
    grabberServo.write(0); 
  }
}
