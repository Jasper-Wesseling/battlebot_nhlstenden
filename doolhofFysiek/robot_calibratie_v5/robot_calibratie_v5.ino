#include <Adafruit_NeoPixel.h>

const int NUM_PIXELS = 4; // Number of NeoPixels in the strip
const int PIXEL_PIN = 18;  // Pin connected to the NeoPixels
// Create NeoPixel object
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// gripper
const int GRIPPER_PIN = 7;
// line sensor pins
int lineSensors[] = {A0, A1, A2, A4, A3, A5, A6, A7};
int lineFollowCorrection[] = {200, 160, 80, 40, 40, 80, 160, 200};
int sensorTarget = 930;

// timing
unsigned long previousMillis = 0UL;
unsigned long interval = 30UL;

// Motor setup
int MotorLeft_1 = 9;
int MotorLeft_2 = 10;
int MotorRight_1 = 5;
int MotorRight_2 = 6;
int MotorCorrection = 0;

// Button setup
int button1 = 4;
int lastButtonState = HIGH; // Track last button state
bool sequenceRunning = false; // Track if the movement sequence is active

// wheel sensor 1 and 2 and rotation count
// ### important 1 and 2 port are used because you can attach a interupt on these ports ###
int ws1 = 3;
int ws2 = 2;
unsigned long rotationLeft = 0;
unsigned long rotationRight = 0;
unsigned long rotationLeftGoal = 0;
unsigned long rotationRightGoal = 0;
volatile bool move = false;

// sonic sensor
const int trigPin = 13;
const int echoPin = 12;
const int servo = 11;



void setup() {
  // put your setup code here, to run once:
  pinMode(GRIPPER_PIN, OUTPUT);
  
  Serial.begin(9600);
  pinMode(button1, INPUT_PULLUP); // Use internal pull-up resistor
  // for all the line sensors
  for (int i = 0; i < 8; i++) {
    pinMode(lineSensors[i], INPUT);
  }
  pinMode(MotorRight_1, OUTPUT);
  pinMode(MotorRight_2, OUTPUT);
  pinMode(MotorLeft_1, OUTPUT);
  pinMode(MotorLeft_2, OUTPUT);
  MotorsStop();

  // interupt on input speedsensort to accuretly measure rotation
  attachInterrupt(digitalPinToInterrupt(ws1), increaseWheelSensorCountLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ws2), increaseWheelSensorCountRight, CHANGE);

  // sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {
  //unsigned long currentTime = millis();
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) { 
    // Read button state
    int buttonState = digitalRead(button1);

    // Detect button press (transition from HIGH to LOW)
    if (buttonState == LOW && lastButtonState == HIGH) {
      Serial.println("begin");
      moveGripper(1750);


    }
    lastButtonState = buttonState; // Update button state
  
  previousMillis = currentMillis;
  }

}

// Function to control the gripper
// Function is made with millis() in mind so that it doesnt overload/clog up the functions that should actually run.
void moveGripper(int pulse){
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + 20;  // 20ms interval for servo
  }
}

void startDoolhof() {
  turnWheelByRotation(55, 255);
  turnRightByRotation(14, 255);
  turnWheelByRotation(35, 255);
}

void moveToPreffertPosistion(String preffertPosistion, int distence) {
  if (preffertPosistion == "Forward") {
    turnWheelByRotation(distence, 255);
  } else if (preffertPosistion == "Left") { 
    turnLeftByRotation(14, 255);
    moveToPreffertPosistion("Forward", distence);
  } else if (preffertPosistion == "Right") {
    turnRightByRotation(14, 255);
    moveToPreffertPosistion("Forward", distence);
  }
}


void turnWheelByRotation(int rotationCount, int speed) {
  TurnMotorLeft(speed);
  TurnMotorRight(speed);
  
  unsigned long previousMillis = millis();
  unsigned long previousMillisForCheck = millis();
  const unsigned long interval = 30; // Interval in milliseconds
  const unsigned long intervalForCheck = 100; // Interval in milliseconds

  rotationLeftGoal = rotationLeft + rotationCount;
  rotationRightGoal = rotationRight + rotationCount;

  move = true;

  while (move) { // Main movement loop
    unsigned long currentMillis = millis(); // Update current time continuously

    // Interval for motor control adjustments
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      noInterrupts();

      if (rotationLeft > rotationRight) {
        TurnMotorLeft(speed);
        TurnRightWithCurve(speed, speed - 20);
      }

      if (rotationLeft < rotationRight) {
        TurnMotorRight(speed);
        TurnLeftWithCurve(speed, speed - 20);
      }

      if (rotationLeft == rotationRight) {
        TurnMotorLeft(speed);
        TurnMotorRight(speed);
      }  

      
      interrupts();
    }

    // Separate interval for checking condition every 200ms
    if (currentMillis - previousMillisForCheck >= intervalForCheck) {
      previousMillisForCheck = currentMillis;
      moveServo(90);
      delayMicroseconds(2);
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      int duration = pulseIn(echoPin, HIGH);
      int distance = duration * 0.034 / 2;

      if (distance < 10) {
        Serial.println("aaaaaaaaaaaaaahhhhhh");
        MotorsStop();
        move = false;
        rotationLeftGoal = 0;
        rotationRightGoal = 0;
      }
    }
  }

  MotorsStop();
}


void turnLeftByRotation(int rotationCount, int speed) {
  TurnMotorLeft(speed);
  TurnMotorReverseRight(speed);
  
  unsigned long startTime = millis();
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  const unsigned long interval = 30; // Interval in milliseconds

  rotationLeftGoal = rotationLeft + rotationCount;
  rotationRightGoal = rotationRight + rotationCount;

  move = true;

  // while (millis() - startTime < 5000) { // Run for 5 seconds
  // !(rotationLeft >= goalLeft) || !(rotationRight != goalRight)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    noInterrupts();
    while (move) { // Run for 5 seconds  
      // interrupts();
      // Serial.println(rotationLeft);
      // Serial.println(rotationRight);
      if (rotationLeft > rotationRight) {
        TurnMotorLeft(speed - 20);
        TurnMotorReverseRight(speed);
      }

      if (rotationLeft < rotationRight) {
        TurnMotorReverseRight(speed - 20);
        TurnMotorLeft(speed);
      }

      if (rotationLeft == rotationRight) {
        TurnMotorLeft(speed);
        TurnMotorReverseRight(speed);
      }   
      interrupts();
    }
  }
 
  MotorsStop();
}


void turnRightByRotation(int rotationCount, int speed) {
  TurnMotorRight(speed);
  TurnMotorReverseLeft(speed);
  
  unsigned long startTime = millis();
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  const unsigned long interval = 30; // Interval in milliseconds

  rotationLeftGoal = rotationLeft + rotationCount;
  rotationRightGoal = rotationRight + rotationCount;

  move = true;

  // while (millis() - startTime < 5000) { // Run for 5 seconds
  // !(rotationLeft >= goalLeft) || !(rotationRight != goalRight)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    noInterrupts();
    while (move) { // Run for 5 seconds  
      // interrupts();
      // Serial.println(rotationLeft);
      // Serial.println(rotationRight);
      if (rotationLeft > rotationRight) {
        TurnMotorRight(speed - 20);
        TurnMotorReverseLeft(speed);
      }

      if (rotationLeft < rotationRight) {
        TurnMotorReverseLeft(speed - 20);
        TurnMotorRight(speed);
      }

      if (rotationLeft == rotationRight) {
        TurnMotorRight(speed);
        TurnMotorReverseLeft(speed);
      }   
      interrupts();
    }
  }
 
  MotorsStop();
}

int GetDistance(String direction) {
  if (direction == "Forward") {
    moveServo(90);
  } else if (direction == "Left") {
    moveServo(0);
  } else if (direction == "Right") {
    moveServo(180);
  }
  
  delay(1000);
  // timing stuff 
  volatile long duration;
  volatile int distance;

  // noInterrupts();

  // gewooon for loopje doen 3 keer
  int readings[3];
  for (int i = 0; i < 3; i++) {
    // Trigger the ultrasonic sensor
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // removed the max wait time because we ran into timing issues
    duration = pulseIn(echoPin, HIGH);
    // Convert to distance in cm
    distance = duration * 0.034 / 2;
    readings[i] = distance;
    delay(200);
  }  
  
  // time out returns 0
  // Find min and max values from readings
  int minVal = min(readings[0], min(readings[1], readings[2]));
  int maxVal = max(readings[0], max(readings[1], readings[2]));

  // Define a threshold (adjust this based on testing)
  int threshold = 2;  

  // If the difference between max and min readings is within the threshold, return the average
  if ((maxVal - minVal) <= threshold) {
    Serial.println((readings[0] + readings[1] + readings[2]) / 3);
    return (readings[0] + readings[1] + readings[2]) / 3; 
  } else {
    Serial.println(-1);
    return -1;  // Return -1 to indicate an invalid reading
  }
}



void increaseWheelSensorCountLeft() {
  rotationLeft++;
  if (move == true &&  (rotationLeft >= rotationLeftGoal && rotationRight >= rotationRightGoal)) { move = false;}
}

void increaseWheelSensorCountRight() {
  rotationRight++;
  if (move == true &&  (rotationLeft >= rotationLeftGoal && rotationRight >= rotationRightGoal)) { move = false;}
}

void moveServo(int angle)
{
  for (int i = 0; i < 10; i++)
  {
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    digitalWrite(servo, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servo, LOW);
  }
}

// ###### motor controll ######
void TurnMotorLeft(int speed) {
  analogWrite(MotorLeft_1, speed);
  analogWrite(MotorLeft_2, 0);
}

void TurnMotorRight(int speed) {
  // motorcorrection
  speed = speed - MotorCorrection;
  analogWrite(MotorRight_1, speed);
  analogWrite(MotorRight_2, 0);
}

void TurnMotorReverseLeft(int speed) {
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorLeft_2, speed);
}

void TurnMotorReverseRight(int speed) {
  // motor correction
  speed = speed - MotorCorrection;
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
  TurnMotorRight(speed);
  TurnMotorReverseLeft(speed);
}

void TurnRight(int speed) {
  TurnMotorReverseRight(speed);
  TurnMotorLeft(speed);
}

void MotorsStop() {
  analogWrite(MotorRight_1, 0);
  analogWrite(MotorRight_2, 0);
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorLeft_2, 0);

}

void TurnLeftWithCurve(int speed, int angle) {
  TurnMotorRight(speed - angle);
  TurnMotorLeft(speed);
}

void TurnRightWithCurve(int speed, int angle) {
  TurnMotorRight(speed);
  TurnMotorLeft(speed - angle);
}
