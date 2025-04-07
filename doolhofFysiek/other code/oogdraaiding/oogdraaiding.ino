#include <Servo.h>

int MotorL_1 = 9;
int MotorL_2 = 10;
int MotorR_1 = 5;
int MotorR_2 = 6;
int OogMotor = 11; 

const int trigPin = 13;
const int echoPin = 12;

const int shortDistance = 12;

Servo myServo;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  
  myServo.attach(OogMotor);
}

void loop() {
  int distance = getDistance(trigPin, echoPin);

  Serial.print("Measured Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance <= shortDistance) {
    Serial.println("Object detected! Moving servo.");
    myServo.write(0);
  } else {
    myServo.write(90);
  }

  delay(500); 
}


int getDistance(int trigPin, int echoPin) {
  long duration;
  int distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

// 0 rechts
// 90 rechtdoor
// 180 links