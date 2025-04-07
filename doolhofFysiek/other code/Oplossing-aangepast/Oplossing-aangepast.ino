#include <Adafruit_NeoPixel.h>

#define NUM_LED 4  // Number of NeoPixels in the strip
#define NeoLED 8   // Pin connected to the NeoPixels
#define DELAY_TIME 500    // Delay time between LED blinks

// Create NeoPixel object
Adafruit_NeoPixel strip(NUM_LED, NeoLED, NEO_GRB + NEO_KHZ800);

// Motor pins
const int A1_MOTOR_PIN = 6;
const int A2_MOTOR_PIN = 5;
const int B1_MOTOR_PIN = 10;
const int B2_MOTOR_PIN = 9;
const int R1_ROTATION_PIN = 2;
const int R2_ROTATION_PIN = 3;
int deadPulses;
int currentPulse;
// IR pins
const int IR_PIN_ONE = A0;
const int IR_PIN_FOUR = A1;
const int IR_PIN_FIVE = A2;
const int IR_PIN_EIGHT = A3;
// Servo pins
const int SERVO_PIN = 11;
const int TRIGGER_PIN = 13;
const int ECHO_PIN = 12;
// Rotation intergers
int r1Rotations = 0;
int r2Rotations = 0;
// Approx*
const int ONE_CM_IN_ROTATIONS = 2;
// echo sensor variables
long duration;
int distance;
// Variables for determining pulse movement
int pulsesToMove;
// Keep track of whether robot moves or not
int currentAmountOfPulses;
int noMoveCounter;
boolean isPreviousVoid = false;
// Use these as reference points
double rightDistance = 0;
double leftDistance = 0;
double differenceInDistance;
boolean isInMiddle = false;
boolean isStartupDone = false;
const int IR_COLOR_BLACK = 900;
const int GRIPPER_PIN = 7;

// Setup function
void setup()
{
  Serial.begin(9600);
  strip.begin();  // Corrected from 'strip.begin();'
  strip.show();   // Initialize all pixels to 'off'

      // Send an initial pulse and ignore the first reading
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms
    Serial.println("First reading ignored...");
}

// Function to move the robot forward
void moveForward(int leftSpeed, int rightSpeed)
{
  strip.setPixelColor(2, strip.Color(255, 255, 255));
  strip.setPixelColor(3, strip.Color(255, 255, 255));
  strip.setPixelColor(1, strip.Color(0, 50, 0));
  strip.setPixelColor(0, strip.Color(0, 50, 0));
  strip.show();  // Update LED strip
  
  analogWrite(A1_MOTOR_PIN, 0);
  analogWrite(A2_MOTOR_PIN, leftSpeed);
  analogWrite(B1_MOTOR_PIN, 0);
  analogWrite(B2_MOTOR_PIN, rightSpeed);


  // Serial communication
  Serial.begin(9600);
 strip.begin();
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
  pinMode(IR_PIN_ONE, INPUT);
  pinMode(IR_PIN_FOUR, INPUT);
  pinMode(IR_PIN_FIVE, INPUT);
  pinMode(IR_PIN_EIGHT, INPUT);
  pinMode(GRIPPER_PIN, OUTPUT);
  // Initialization routine
  for (int i = 0; i < 3; i++){
    while (getDistanceFromPulse() > 29)
    {
      delay(1);
    }
  }
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
  while (analogRead(IR_PIN_ONE) > IR_COLOR_BLACK){
    moveForward(245,255);
  }
  while (analogRead(IR_PIN_ONE) < IR_COLOR_BLACK){
    moveForward(245,255);
  }
  while (analogRead(IR_PIN_ONE) > IR_COLOR_BLACK){
    moveForward(245,255);
  }
  //CLOSE THE SERVO
  isStartupDone = true;
  moveGripper(37);
  moveForward(0 , 255);
  delay(700);
  while (analogRead(IR_PIN_FIVE) < IR_COLOR_BLACK){
    delay(1);
  }
  stopMoving();
  moveForward(255,245);
  delay(1000);
  stopMoving();
}

// Interrupt routine for rotation pin R1
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
  moveGripper(37);
}

// Interrupt routine for rotation pin R2
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
  moveGripper(37);
}

// Check if R1 rotation is dead
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

// Main loop function
void loop()
{
  // IR sensor readings and movement control based on sensor readings
  if(analogRead(IR_PIN_ONE) > IR_COLOR_BLACK)
  {
    moveForward(240,200);
  }
  if(analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK)
  {
    moveForward(240,220);
  }
  if(analogRead(IR_PIN_FIVE) > IR_COLOR_BLACK)
  {
    moveForward(220,240);
  }
  if(analogRead(IR_PIN_EIGHT) > IR_COLOR_BLACK)
  {
    moveForward(200,240);
  }
  if (analogRead(IR_PIN_ONE) > IR_COLOR_BLACK && analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK && analogRead(IR_PIN_FIVE) > IR_COLOR_BLACK && analogRead(IR_PIN_EIGHT) > IR_COLOR_BLACK )
  {
    stopMoving();
    delay(250);
    moveGripper(100);
    exit(0);
  }
  
  
// Called when the robot doesnt know how far it can drive.
  // Basically the initial push for the robot to start
  if (pulsesToMove == 0)
  {
    // Move the servo forward
    moveServo(90);
    delay(250);
    // scan for distance
    long cmToMove = getDistanceFromPulse();
    cmToMove = constrain(cmToMove, 0, 150);
    // Calculate distance if pulses
    pulsesToMove = (cmToMove * ONE_CM_IN_ROTATIONS) - 1;
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
  // End of main loop 
  else if (r1Rotations >= (pulsesToMove - 3))
  {
    stopMoving();
    Serial.println("Approx CM moved:");
    Serial.print(r1Rotations / ONE_CM_IN_ROTATIONS);
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
    while (r1Rotations < 24)
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
void turnLeft()
{
  delay(200);
  // This is called incase of the robot being against the wall in front.
  moveBackward(220, 220);
  delay(200);
  stopMoving();
  delay(100);
  r2Rotations = 0;
  moveServo(0);

  // Turn the robot left
  while (r2Rotations < 28)
  {
    moveLeft(0, 250);
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
void turnRight()
{
  delay(200);
  // Move backwards to counter-act the forward creeps because they could work against the turn.
  moveBackward(220, 220);
  delay(200);
  stopMoving();
  delay(100);
  r1Rotations = 0;
  // Actually turn right here.
  while (r1Rotations < 29)
  {
    moveRight(220, 0);
    checkIfR1Dead(true);
    Serial.println(r1Rotations);
    sendPulse();
    if (distance < 14) 
    {
      break;
    }
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
 strip.setPixelColor(3,strip.Color(0, 50, 0));
 strip.setPixelColor(2,strip.Color(64, 255, 0));
 strip.setPixelColor(1,strip.Color(64, 255, 0));
 strip.setPixelColor(0,strip.Color(0, 50, 0));
 strip.show();
  analogWrite(A1_MOTOR_PIN, 0);
  analogWrite(A2_MOTOR_PIN, leftSpeed);
  analogWrite(B1_MOTOR_PIN, rightSpeed);
  analogWrite(B2_MOTOR_PIN, 0);
}

// Function to move the robot left
void moveLeft(int leftSpeed, int rightSpeed)
{
 strip.setPixelColor(3,strip.Color(64, 255, 0));
 strip.setPixelColor(2,strip.Color(0, 50, 0));
 strip.setPixelColor(1,strip.Color(0, 50, 0));
 strip.setPixelColor(0,strip.Color(64, 255, 0));
 strip.show();
  analogWrite(A1_MOTOR_PIN, leftSpeed);
  analogWrite(A2_MOTOR_PIN, 0);
  analogWrite(B1_MOTOR_PIN, 0);
  analogWrite(B2_MOTOR_PIN, rightSpeed);
}

// Function to control the servo motor
void moveServo(int angle)
{
  for (int i = 0; i < 10; i++)
  {
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// Function to send ultrasonic pulses
void sendPulse() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
    if (duration == 0) {
        Serial.println("No echo received!");
        distance = -1;  // Set distance to -1 when no signal
    } else {
        distance = duration * 0.034 / 2;
    }

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


// Function to move the robot backward
void moveBackward(int leftSpeed, int rightSpeed)
{
 strip.setPixelColor(2,strip.Color(0, 50, 0));
 strip.setPixelColor(3,strip.Color(0, 50, 0));
 strip.setPixelColor(1,strip.Color(255, 255, 255));
 strip.setPixelColor(0,strip.Color(255, 255, 255));
 strip.show();
  analogWrite(A1_MOTOR_PIN, leftSpeed);
  analogWrite(A2_MOTOR_PIN, 0);
  analogWrite(B1_MOTOR_PIN, rightSpeed);
  analogWrite(B2_MOTOR_PIN, 0);
}

// Function to stop all robot movement
void stopMoving()
{
 strip.setPixelColor(0,strip.Color(0, 255, 0));
 strip.setPixelColor(1,strip.Color(0, 255, 0));
 strip.setPixelColor(2,strip.Color(50, 50, 50));
 strip.setPixelColor(3,strip.Color(50, 50, 50));
 strip.show();
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

// Function to control the gripper
// Function is made with millis() in mind so that it doesnt overload/clog up the functions that should actually run.
void moveGripper(int angle)
{
  static long refreshTime;
  if(millis() > refreshTime && isStartupDone)
  {
    for (int i = 0; i < 10; i++)
    {
      int pulseWidth = map(angle, 0, 180, 544, 2400);

      digitalWrite(GRIPPER_PIN, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(GRIPPER_PIN, LOW);
    }
    refreshTime = millis() + 500;
  }
}
