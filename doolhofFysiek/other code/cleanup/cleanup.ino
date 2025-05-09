/*
-----------------------------Team Informatie------------------------------
  RobotNummer: BB004
  Robot naam: Muisbertus
  Groep: IC-INF-1C (IDrunkDrive)
  Contributors: Thomas scholtens & Jasper Wesseling

  COPYRIGHT: © 2025 | IDrunkDrive


  ╔════════════════════════════════════════════════════╗
  ║               ARDUINO PIN CONFIGURATION           ║
  ╠════════════════════╦══════════════════════════════╣
  ║      ANALOG PINS   ║        FUNCTION              ║
  ╠════════════════════╬══════════════════════════════╣
  ║ A0                 ║ Sensor 1 - Infrared Line     ║
  ║ A1                 ║ Sensor 2 - Infrared Line     ║
  ║ A2                 ║ Sensor 3 - Infrared Line     ║
  ║ A3                 ║ Sensor 4 - Infrared Line     ║
  ║ A4                 ║ Sensor 5 - Infrared Line     ║
  ║ A5                 ║ Sensor 6 - Infrared Line     ║
  ║ A6                 ║ Sensor 7 - Infrared Line     ║
  ║ A7                 ║ Sensor 8 - Infrared Line     ║
  ╠════════════════════╬══════════════════════════════╣
  ║     DIGITAL PINS   ║        FUNCTION              ║
  ╠════════════════════╬══════════════════════════════╣
  ║ 2                  ║ R2_ROTATION_PIN - Speed Sensor  ║
  ║ 3                  ║ R1_ROTATION_PIN - Speed Sensor  ║
  ╠════════════════════╬══════════════════════════════╣
  ║   MOTOR CONTROL    ║                              ║
  ╠════════════════════╬══════════════════════════════╣
  ║ 5                  ║ B2_MOTOR_PIN                 ║
  ║ 6                  ║ B1_MOTOR_PIN                 ║
  ║ 9                  ║ A2_MOTOR_PIN                 ║
  ║ 10                 ║ A1_MOTOR_PIN                 ║
  ╠════════════════════╬══════════════════════════════╣
  ║  LEDS & SERVOS     ║                              ║
  ╠════════════════════╬══════════════════════════════╣
  ║ 7                  ║ GRIPPER_PIN - Servo Gripper  ║
  ║ 8                  ║ PIXEL_PIN - Corner Lamps     ║
  ║ 11                 ║ SERVO_PIN - Ultrasonic Rot.  ║
  ╠════════════════════╬══════════════════════════════╣
  ║ ULTRASONIC SENSOR  ║                              ║
  ╠════════════════════╬══════════════════════════════╣
  ║ 12                 ║ ECHO_PIN                     ║
  ║ 13                 ║ TRIGGER_PIN                  ║
  ╚════════════════════╩══════════════════════════════╝
*/

// libery for the LED
#include <Adafruit_NeoPixel.h>

/* 
########################
  pint defines
########################
*/
// LED pins
#define NUM_PIXELS 4 // Number of NeoPixels in the strip
#define PIXEL_PIN 8  // Pin connected to the NeoPixels

// Motor pins and speed sensors
#define A1_MOTOR_PIN 10
#define A2_MOTOR_PIN 9
#define B1_MOTOR_PIN 6
#define B2_MOTOR_PIN 5
#define R1_ROTATION_PIN 3
#define R2_ROTATION_PIN 2

// IR pins
#define IR_PIN_ZERO A0
#define IR_PIN_FOUR A4
#define IR_PIN_THREE A3
#define IR_PIN_SEVEN A7

// Servo pins
#define SERVO_PIN 11
#define GRIPPER_PIN 7

// Sonic sensor pin
#define TRIGGER_PIN 13
#define ECHO_PIN 12

// Rotation intergers
int r1Rotations = 0;
int r2Rotations = 0;

// Readings from sonic sensor in cm to approximately rotation wheel to cover distence
const int ZERO_CM_IN_ROTATIONS = 2;

// sonic sensor variables
long duration;
int distance;

// Variables for determining pulse movement
int pulsesToMove;

// Keep track of whether robot moves or not
int currentAmountOfPulses;
int noMoveCounter;
boolean isPreviousVoid = false;

// reference points
double rightDistance = 0;
double leftDistance = 0;
double differenceInDistance;
boolean isInMiddle = false;

// Define the color black threshold of IR sensor
const int IR_COLOR_BLACK = 900;

// Creating the LED object
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// int for keeping track of movement or no movement of the wheels
int deadPulses;
int currentPulse;



/* 
########################
  Setup
########################
*/
void setup()
{
  // Serial communication
  Serial.begin(9600);
  pixels.begin();
  // Setup rotation pins and attach interrupt routine
  pinMode(R1_ROTATION_PIN, INPUT);
  pinMode(R2_ROTATION_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(R1_ROTATION_PIN), rotateR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R2_ROTATION_PIN), rotateR2, CHANGE);
  // Echo pins
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Motor pins
  pinMode(A1_MOTOR_PIN, OUTPUT);
  pinMode(A2_MOTOR_PIN, OUTPUT);
  pinMode(B1_MOTOR_PIN, OUTPUT);
  pinMode(B2_MOTOR_PIN, OUTPUT);
  pinMode(IR_PIN_ZERO, INPUT);
  pinMode(IR_PIN_FOUR, INPUT);
  pinMode(IR_PIN_THREE, INPUT);
  pinMode(IR_PIN_SEVEN, INPUT);
  pinMode(GRIPPER_PIN, OUTPUT);
  
  startOfMaze();
}



/* 
########################
  The main loop
########################
*/
void loop()
{
  // close the gripper
  moveGripper(1100);

  // IR sensor readings and movement control based on sensor readings
  if(analogRead(IR_PIN_ZERO) > IR_COLOR_BLACK)
  {
    moveForward(240,200);
  }
  if(analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK)
  {
    moveForward(240,220);
  }
  if(analogRead(IR_PIN_THREE) > IR_COLOR_BLACK)
  {
    moveForward(220,240);
  }
  if(analogRead(IR_PIN_SEVEN) > IR_COLOR_BLACK)
  {
    moveForward(200,240);
  }
  if (analogRead(IR_PIN_ZERO) > IR_COLOR_BLACK && analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK && analogRead(IR_PIN_THREE) > IR_COLOR_BLACK && analogRead(IR_PIN_SEVEN) > IR_COLOR_BLACK )
  {
    // Stop the code because the end is detected
    stopMoving();
    delay(250);
    moveGripper(100);
    exit(0);
  }
  
  /* 
  ########################
    Solve maze
  ########################
  */
  // Called when the robot doesnt know how far it can drive.
  // Basically the initial push for the robot to start
  if (pulsesToMove == 0)
  {
    // Move the servo forward
    moveServo(96);
    delay(250);
    // scan for distance
    long cmToMove = getDistanceFromPulse();
    cmToMove = constrain(cmToMove, 0, 150);
    // Calculate distance if pulses
    pulsesToMove = (cmToMove * ZERO_CM_IN_ROTATIONS) - 1;
    if (cmToMove < 12)
    {
      moveServo(180);
      delay(250);
      sendPulse();
      if (distance >= 30)
      {
        stopMoving();
        delay(150);
        turnLeft();
      }
      else if (distance < 25)
      {
        checkSurrounding();
        turnAround();
      }
    }
    moveServo(0); // Servo naar rechts draaien
    delay(250);
  }
  // End of main loop pretty much
  else if (r1Rotations >= (pulsesToMove - 3))
  {
    stopMoving();
    Serial.println("Approx CM moved:");
    Serial.print(r1Rotations / ZERO_CM_IN_ROTATIONS);
    pulsesToMove = 0;
    r1Rotations = 0;
    // And back to start from here
    
  }
  // Is essentially the main "hub" for the code.
  // Anything in here should make the robot move or turn.
  else
  {
    
    sendPulse();
    
    // Should avoid accidental run in with old wall
    if (distance > 30)
      isPreviousVoid = false;
    
    // Make the robot drive forward and turn right.
    if (distance > 35 && !isPreviousVoid)
    {
      int creepForwardRotations = r1Rotations + 29;
      Serial.println("Creeping forward");
      // Makes sure it doesnt bonk head into right wall
      while (r1Rotations < creepForwardRotations)
      {
        moveForward(200, 220);
        checkIfR1Dead(true);
        Serial.print("Forward creep rotation: ");
        Serial.println(r1Rotations);
      }
      stopMoving();
      turnRight();
    }
    adjustDirection();
    checkIfR1Dead(true);
  }
}

// Function to check surrounding obstacles
void checkSurrounding()
{
  moveServo(1);
  delay(500);
  sendPulse();
  rightDistance = (double)distance;
  moveServo(180);
  delay(500);
  sendPulse();
  leftDistance = (double)distance;
  differenceInDistance = rightDistance / leftDistance;
  if (differenceInDistance > 0.8 && differenceInDistance < 1.2)
  {
    isInMiddle = true;
    Serial.println("It's probably in the middle");
  }
  else
  {
    isInMiddle = false;
  }
}

/* 
########################
  movment of the robot
########################
*/

// Function to execute a turn-around maneuver
void turnAround()
{
  // If its almost perfectly in the middle it can turn a full 180 without leverage.
  if (isInMiddle)
  {
    r1Rotations = 0;
    // First 90 degree turn.
    while (r1Rotations < 36)
    {
      moveBackward(220, 0);
      checkIfR1Dead(false);
      Serial.println(r1Rotations);
    }
    stopMoving();
    delay(150);
    moveBackward(220, 220);
    delay(750);
    stopMoving();
    delay(150);
    r2Rotations = 0;
    moveServo(0);
    // second 90 degree turn. it does check for a wall if needed.
    while (r2Rotations < 28)
    {
      moveForward(0, 220);
      checkIfR2Dead(true);
      Serial.println(r2Rotations);
      sendPulse();
      if(distance < 10)
        break;
    }
    stopMoving();
    delay(100);
    moveServo(180);
  }
  // Execute other turn if its got the space to go left.
  else if (leftDistance > rightDistance)
  {
    r1Rotations = 0;
    // First 60 degrees
    while (r1Rotations < 20)
    {
      moveBackward(210, 0);
      checkIfR1Dead(false);
      Serial.println(r1Rotations);
    }
    delay(150);
    r2Rotations = 0;
    // Second 60 degrees
    while (r2Rotations < 20)
    {
      moveForward(0, 210);
      checkIfR2Dead(true);
      Serial.println(r2Rotations);
    }
    delay(150);
    r1Rotations = 0;
    moveServo(180);
    // Third 60 degrees
    // Also checks for wall.
    while (r1Rotations < 10)
    {
      moveBackward(210, 0);
      checkIfR1Dead(false);
      Serial.println(r1Rotations);
      sendPulse();
      if(distance < 12)
        break;
    }
    delay(150);
    stopMoving();
  }
  // Execute other turn if its got the space to go right.
  else if (leftDistance < rightDistance)
  {
    r2Rotations = 0;
    // First 60 degrees
    while (r2Rotations < 24)
    {
      moveBackward(0, 210);
      checkIfR2Dead(false);
      Serial.println(r2Rotations);
    }
    delay(150);
    r1Rotations = 0;
    // Second 60 degrees
    while (r1Rotations < 20)
    {
      moveForward(210, 0);
      checkIfR1Dead(true);
      Serial.println(r1Rotations);
    }
    delay(150);
    r2Rotations = 0;
    // Third 60 degrees
    // Also checks for wall.
    while (r2Rotations < 20)
    {
      moveServo(0);
      sendPulse();
      if(distance < 12)
      {
        break;
      }
      moveBackward(0, 210);
      checkIfR2Dead(false);
      Serial.println(r2Rotations);

    }
    delay(150);
    stopMoving();
  }
}

// A more in depth function to turn left.
void turnLeft() {
  delay(200);
  moveBackward(220, 220); // Reverse slightly before turning
  delay(200);
  stopMoving();
  delay(100);
  r2Rotations = 0;
  moveServo(0);

  // quick max power boost
  moveLeft(0, 255);  
  delay(400);

  // Reduce speed to normal turning speed
  while (r2Rotations < 28) {
    moveLeft(0, 220);
    checkIfR2Dead(true);
    sendPulse();
    if (distance < 13)
      break;
  }
  stopMoving();
  r2Rotations = 0;
  delay(200);
  moveBackward(220, 220);
  delay(200);
  stopMoving();
  pulsesToMove = 0;
  r1Rotations = 0;
  r2Rotations = 0;
}


// A more in depth function to turn turn
void turnRight() {
  delay(200);
  moveBackward(220, 220);
  delay(200);
  stopMoving();
  delay(100);
  r1Rotations = 0;

  // quick max power boost
  moveRight(255, 0);  
  delay(400);

  // Reduce speed to normal turning speed
  while (r1Rotations < 28) {
    moveRight(220, 0);
    checkIfR1Dead(true);
    Serial.println(r1Rotations);
    sendPulse();
    if (distance < 14) 
      break;
  }
  stopMoving();
  delay(200);
  moveBackward(220, 220);
  delay(200);
  isPreviousVoid = true;
  stopMoving();
  pulsesToMove = 0;
  r1Rotations = 0;
  r2Rotations = 0;
  delay(200);
}


// Function to move the robot right
void moveRight(int leftSpeed, int rightSpeed)
{
  pixels.setPixelColor(3, pixels.Color(0, 50, 0));
  pixels.setPixelColor(2, pixels.Color(64, 255, 0));
  pixels.setPixelColor(1, pixels.Color(64, 255, 0));
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  analogWrite(A1_MOTOR_PIN, 0);
  analogWrite(A2_MOTOR_PIN, leftSpeed);
  analogWrite(B1_MOTOR_PIN, rightSpeed);
  analogWrite(B2_MOTOR_PIN, 0);
}

// Function to move the robot left
void moveLeft(int leftSpeed, int rightSpeed)
{
  pixels.setPixelColor(3, pixels.Color(64, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 50, 0));
  pixels.setPixelColor(1, pixels.Color(0, 50, 0));
  pixels.setPixelColor(0, pixels.Color(64, 255, 0));
  pixels.show();
  analogWrite(A1_MOTOR_PIN, leftSpeed);
  analogWrite(A2_MOTOR_PIN, 0);
  analogWrite(B1_MOTOR_PIN, 0);
  analogWrite(B2_MOTOR_PIN, rightSpeed);
}


/* 
########################
  sonic sensor control
########################
*/

// Function to send ultrasonic pulses
void sendPulse()
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// Function to get average distance from ultrasonic pulse
// This function is called so that we get a more accurate reading.
int getDistanceFromPulse()
{
  int avaragePulse = 0;
  for (int i = 0; i < 5; i++)
  {
    sendPulse();
    avaragePulse += distance;
    delay(10);
  }
  avaragePulse /= 5;
  Serial.print("Approx distance able to travel: ");
  Serial.print(avaragePulse);
  Serial.println(" cm");
  return avaragePulse;
}

/* 
########################
  Servo motors control
########################
*/

// Move the gripper in the front to grip the cone
void moveGripper(int pulse) {
  static unsigned long lastTime = 0;
  static int lastPulse = 1500;  // Default: Neutral servo position

  if (pulse > 0) {
    lastPulse = pulse;  // Update to new pulse width
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 20) {  // Ensure 50Hz signal
    lastTime = currentTime;

    // Generate servo pulse
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(lastPulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}

// Function to control the servo motor for the sonic sensor
void moveServo(int angle)
{
  delay(100);
  for (int i = 0; i < 10; i++)
  {
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
  }
}

/* 
########################
  Posions the robot 
  at the start of the maze
########################
*/
void startOfMaze() {
  // read pulses
  for (int i = 0; i < 3; i++){
    while (getDistanceFromPulse() > 29)
    {
      delay(1);
    }
  }

  // when detected wait for other robot to clear the start zone and show wait signal 
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(5000); 

  // when detected wait for other robot to clear the start zone
  delay(1000); 

  // try to follow the line
  for (int i = 0; i < 3; i++){
    while (analogRead(IR_PIN_FOUR) < IR_COLOR_BLACK){
      delay(1);
      moveForward(245,255);
    }
    while (analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK)
    {
      delay(1);
      moveForward(245,255);
    }
  }
  while (analogRead(IR_PIN_ZERO) > IR_COLOR_BLACK){
    moveForward(245,255);
  }
  while (analogRead(IR_PIN_ZERO) < IR_COLOR_BLACK){
    moveForward(245,255);
  }
  while (analogRead(IR_PIN_ZERO) > IR_COLOR_BLACK){
    moveForward(245,255);
  }

  // close the gripper and turn left
  moveGripper(1100);
  moveForward(0 , 255);
  delay(700);
  // stop than move a little futher in the maze
  stopMoving();
  moveForward(255,245);
  delay(1500);
  stopMoving();
}

/* 
########################
  Motor fuctions
########################
*/
// move forward
void moveForward(int leftSpeed, int rightSpeed)
{
  pixels.setPixelColor(2, pixels.Color(255, 255, 255));
  pixels.setPixelColor(3, pixels.Color(255, 255, 255));
  pixels.setPixelColor(1, pixels.Color(0, 50, 0));
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  analogWrite(A1_MOTOR_PIN, 0);
  analogWrite(A2_MOTOR_PIN, leftSpeed);
  analogWrite(B1_MOTOR_PIN, 0);
  analogWrite(B2_MOTOR_PIN, rightSpeed);
}

// Move backwards
void moveBackward(int leftSpeed, int rightSpeed)
{
  pixels.setPixelColor(2, pixels.Color(0, 50, 0));
  pixels.setPixelColor(3, pixels.Color(0, 50, 0));
  pixels.setPixelColor(1, pixels.Color(255, 255, 255));
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  analogWrite(A1_MOTOR_PIN, leftSpeed);
  analogWrite(A2_MOTOR_PIN, 0);
  analogWrite(B1_MOTOR_PIN, rightSpeed);
  analogWrite(B2_MOTOR_PIN, 0);
}

// stop moving
void stopMoving()
{
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(50, 50, 50));
  pixels.setPixelColor(3, pixels.Color(50, 50, 50));
  pixels.show();
  analogWrite(A1_MOTOR_PIN, 0);
  analogWrite(A2_MOTOR_PIN, 0);
  analogWrite(B1_MOTOR_PIN, 0);
  analogWrite(B2_MOTOR_PIN, 0);
}

// Function to adjust robot direction based on distance readings
void adjustDirection()
{
  if (distance >= 12 && distance <= 14)
  {
    moveForward(245, 255); // Go straight
  }
  else if (distance <= 7) 
  {
    moveForward(160,255);
  }
  else if (distance < 12 && distance > 7)
  {
    moveForward(215, 255);
  }
   else if (distance > 14 && distance < 18)
  {
    moveForward(255, 215);
  }
  else if (distance >= 18 && distance <= 30)
  {
    moveForward(255, 180);
  }
  checkIfR1Dead(true);
}



/* 
########################
  Interrupt routines
########################
*/
// Pin 1
void rotateR1()
{
  noInterrupts();
  static unsigned long timer;
  static bool lastState;
  if(millis() > timer)
  {
    bool state = digitalRead(R1_ROTATION_PIN);
    if(state != lastState)
    {
      r1Rotations++;
      lastState = state;
    }
    timer = millis() + 10;
  }
  interrupts();
}

// Pin 2
void rotateR2()
{
  noInterrupts();
  static unsigned long timer;
  static bool lastState;
  if(millis() > timer)
  {
    bool state = digitalRead(R2_ROTATION_PIN);
    if(state != lastState)
    {
      r2Rotations++;
      lastState = state;
    }
    timer = millis() + 10;
  }
  interrupts();
}

/* 
########################
  Fuction checking if 
  there are no readings 
  from the speed sensors
########################
*/
void checkIfR1Dead(boolean moveBack){
  // Check if pulse is still same when it has to be moving
  if(currentPulse == r1Rotations)
  {
    deadPulses++;
    delay(100);
    Serial.println("Dead Pulse activated");
  }
  else
  // Reset var's when this is called
  {
    currentPulse = r1Rotations;
    deadPulses = 0;
    Serial.println("Dead Pulse reset");
  }
  // Called when robot is stuck, *Should unstuck robot*
  if(deadPulses > 10)
  {
    int oldR1Rotations = r1Rotations;
    stopMoving();
    if(moveBack){
      moveBackward(210,210);
      delay(400);
    }

    // Called depending on boolean state. So forward/backward
    if(!moveBack){
      moveForward(210, 210);
      delay(250);
    }

    stopMoving();
    deadPulses = 0;
  }
}

// Check if R2 rotation is dead
void checkIfR2Dead(boolean moveBack){
  // Check if pulse is still same when it has to be moving
  if(currentPulse == r2Rotations)
  {
    deadPulses++;
    delay(100);
    Serial.println("Dead Pulse activated");
  }
  else
  // Reset var's when this is called
  {
    currentPulse = r2Rotations;
    deadPulses = 0;
    Serial.println("Dead Pulse reset");
  }
  if(deadPulses > 10)
  {
    int oldR2Rotations = r2Rotations;
    stopMoving();
    if(moveBack){
      moveBackward(210,210);
      delay(400);
    }
    
    // Called depending on boolean state. So forward/backward
    if(!moveBack){
      moveForward(210, 210);
      delay(250);
    }
    stopMoving();
    deadPulses = 0;
  }
}