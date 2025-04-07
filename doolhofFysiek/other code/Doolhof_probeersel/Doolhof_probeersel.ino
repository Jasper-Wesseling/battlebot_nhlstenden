#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define NeoLED 8          // The single data pin for all NeoPixels
#define NUM_PIXELS 4      // Total number of NeoPixels (e.g., 4 LEDs)
#define DELAY_TIME 500    // Delay time between LED blinks

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

int MotorLeft_1 = 9;
int MotorLeft_2 = 10;
int MotorRight_1 = 5;
int MotorRight_2 = 6;
const int OogMotor = 11;

const int trigPin = 13;
const int echoPin = 12;

const int shortDistance = 12;  // Minimum distance to consider the path clear
const int MotorCorrection = 10; // Motor correction value

Servo myServo;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(MotorLeft_1, OUTPUT);
  pinMode(MotorLeft_2, OUTPUT);
  pinMode(MotorRight_1, OUTPUT);
  pinMode(MotorRight_2, OUTPUT);

  Serial.begin(9600);

  myServo.attach(OogMotor);
  myServo.write(90);  // Center position at startup

  strip.begin();
  strip.show(); // Initialize all NeoPixels to 'off'

  delay(1000);  // Allow sensor to stabilize at startup
}

void loop() {
  int forwardDistance = getDistance();  // Get the actual forward distance
  Serial.print("Forward Distance: ");
  Serial.println(forwardDistance);

  if (forwardDistance >= shortDistance) {  
    Serial.println("Path is clear, moving forward!");
    MoveForward(255);
    delay(200);  // Move forward for a short time
    return;
  }

  Serial.println("Object detected ahead! Stopping to check sides.");
  MotorsStop();

  // Scan left first
  int leftDistance = getDistanceLeft();
  delay(300);

  if (leftDistance >= shortDistance) {
    Serial.println("Turning Left");
    TurnLeft(255);
    delay(420);
    MotorsStop();
    myServo.write(90);
    return;
  }

  // Scan right if left is blocked
  Serial.println("Scanning right...");
  int rightDistance = getDistanceRight();
  delay(300);

  if (rightDistance >= shortDistance) {
    Serial.println("Turning Right");
    TurnRight(255);
    delay(420);
    MotorsStop();
    myServo.write(90);
    return;
  }

  // If all directions are blocked, move backward
  Serial.println("Blocked on all sides, moving backward.");
  MoveBackwards(200);
  delay(500);
  MotorsStop();
}

int getDistance() {
  return measureDistance();
}

int getDistanceLeft() {
  myServo.write(0);
  delay(500);
  int distance = measureDistance();
  myServo.write(90);
  delay(300);
  return distance;
}

int getDistanceRight() {
  myServo.write(180);
  delay(500);
  int distance = measureDistance();
  myServo.write(90);
  delay(300);
  return distance;
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  Serial.print("Measured Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

// ###### motor control ######
void TurnMotorLeft(int speed) {
  analogWrite(MotorLeft_1, speed);
  analogWrite(MotorLeft_2, 0);
}

void TurnMotorRight(int speed) {
  speed = constrain(speed - MotorCorrection, 0, 255);
  analogWrite(MotorRight_1, speed);
  analogWrite(MotorRight_2, 0);
}

void TurnMotorReverseLeft(int speed) {
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorLeft_2, speed);
}

void TurnMotorReverseRight(int speed) {
  speed = constrain(speed - MotorCorrection, 0, 255);
  analogWrite(MotorRight_1, 0);
  analogWrite(MotorRight_2, speed);
}

// ###### movement ######
void MoveForward(int speed) {
  TurnMotorLeft(speed);
  TurnMotorRight(speed);
}

void MoveBackwards(int speed) {
  TurnMotorReverseLeft(speed);
  TurnMotorReverseRight(speed);
}

void TurnLeft(int speed) {
  TurnMotorReverseRight(speed);
  TurnMotorLeft(speed);
}

void TurnRight(int speed) {
  TurnMotorReverseLeft(speed);
  TurnMotorRight(speed);
}

void MotorsStop() {
  analogWrite(MotorRight_1, 0);
  analogWrite(MotorRight_2, 0);
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorLeft_2, 0);
}

// Nieuwe aangepaste functie voor richtingscorrectie
void adjustDirection()
{
  int actualDistance = getDistance(); // Meet actuele afstand

  if (actualDistance >= 12 && actualDistance <= 14)
  {
    MoveForward(245); // Ga rechtdoor
  }
  else if (actualDistance <= 7) 
  {
    MoveForward(150);  // Slightly shorter move for close distances
  }
  else if (actualDistance < 12 && actualDistance > 7)
  {
    MoveForward(215);  // Smooth adjustment for mid-range
  }
  else if (actualDistance > 14 && actualDistance < 18)
  {
    MoveForward(230);  // Slightly lower value to avoid overshooting
  }
  else if (actualDistance >= 18 && actualDistance <= 30)
  {
    MoveForward(240);  // Lower value for longer distances
  }
}

