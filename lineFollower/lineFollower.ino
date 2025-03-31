#define DEBUG  // Comment this line out to disable debugging

// Neopixel Setup
#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 8
#define NUM_PIXELS 30
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Gripper Setup
#define GRIPPER_SERVO 7
#define GRIPPER_OPEN 1600
#define GRIPPER_CLOSE 1000

// Motor Setup
#define MOTOR_L_A1 10
#define MOTOR_L_A2 9
#define MOTOR_R_B1 6
#define MOTOR_R_B2 5

// Array line sensors setup
#define SENSOR_COUNT 8
const int _lineSensors[SENSOR_COUNT] = { A7, A6, A5, A4, A3, A2, A1, A0 };
int _sensorMin[SENSOR_COUNT];
int _sensorMax[SENSOR_COUNT];

// Ultrasonic sensor
#define TRIG_PIN 13
#define ECHO_PIN 12

// Speed variables
#define MAX_SPEED 255
#define STEADY_SPEED 230
#define SLOW_SPEED 180
#define SLOWER_SPEED 60
#define SLOWEST_SPEED 10
#define SEARCH_SPEED 50

// Extra defines
#define BLACK 900
#define WHITE 600
#define THRESHOLD 600  // Lower threshold for better sensitivity to thin lines
#define DURATION_90_DEGREES_SPIN 600
#define DURATION_MOVEMENT_FORWARDS_START 700
#define DURATION_DELAY_AVOID_OBSTACLE 300
#define DURATION_MOVEMENT_FORWARDS_AVOID_OBSTACLE 1200
#define SOUND_SPEED 0.0343  // Speed of sound in air (cm/µs)
#define BUTTON_PIN 2

float _distance_cm;
long _pulseDuration;

unsigned long _blackStartTime = 0;
bool _isStopping = false;

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_L_A1, OUTPUT);
  digitalWrite(MOTOR_L_A1, LOW);
  pinMode(MOTOR_L_A2, OUTPUT);
  digitalWrite(MOTOR_L_A2, LOW);
  pinMode(MOTOR_R_B1, OUTPUT);
  digitalWrite(MOTOR_R_B1, LOW);
  pinMode(MOTOR_R_B2, OUTPUT);
  digitalWrite(MOTOR_R_B2, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT);

  pinMode(GRIPPER_SERVO, OUTPUT);
  digitalWrite(GRIPPER_SERVO, LOW);

  strip.begin();
  strip.setBrightness(32);
  strip.show();

  // Initialize sensor calibration values
  for (int i = 0; i < SENSOR_COUNT; i++) {
    _sensorMin[i] = 1023;
    _sensorMax[i] = 0;
  }

  // Move forward at start
  motorDrive(MAX_SPEED - 10, MAX_SPEED);
  delay(DURATION_MOVEMENT_FORWARDS_START);

  motorStop();
  delay(100);

  // Close the gripper for a short time
  unsigned long closeStart = millis();
  while (millis() - closeStart < 500)  // Send pulses for 500ms
  {
    gripper(GRIPPER_CLOSE);
  }

  motorDrive(MAX_SPEED - 10, MAX_SPEED);
  delay(DURATION_MOVEMENT_FORWARDS_START);

  // Turn 90 degrees left to start line-following
  motorRotate90DegreesLeft();
}

void loop() {
  gripper(GRIPPER_CLOSE);
  checkObstacle();
  followLine();
}

void motorDrive(int left, int right) {
  analogWrite(MOTOR_L_A1, max(0, -left));
  analogWrite(MOTOR_L_A2, max(0, left));
  analogWrite(MOTOR_R_B1, max(0, -right));
  analogWrite(MOTOR_R_B2, max(0, right));
}

void motorStop() {
  analogWrite(MOTOR_L_A1, 0);
  analogWrite(MOTOR_L_A2, 0);
  analogWrite(MOTOR_R_B1, 0);
  analogWrite(MOTOR_R_B2, 0);
}

void motorRotate90DegreesRight() {
  analogWrite(MOTOR_L_A1, 0);
  analogWrite(MOTOR_L_A2, MAX_SPEED);
  analogWrite(MOTOR_R_B1, MAX_SPEED - 30);
  analogWrite(MOTOR_R_B2, 0);
  delay(DURATION_90_DEGREES_SPIN);
  motorStop();
}

void motorRotate90DegreesLeft() {
  analogWrite(MOTOR_L_A1, MAX_SPEED - 30);
  analogWrite(MOTOR_L_A2, 0);
  analogWrite(MOTOR_R_B1, 0);
  analogWrite(MOTOR_R_B2, MAX_SPEED);
  delay(DURATION_90_DEGREES_SPIN);
  motorStop();
}

// Function to turn on NeoPixel for a specific direction
void setDirectionLights(bool isLeftTurn, bool isRightTurn, bool isMovingStraight) {
  if (isLeftTurn) {
    // Turn on blue for left turn
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 255));  // Blue for left turn
    }
  } else if (isRightTurn) {
    // Turn on blue for right turn
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 255));  // Blue for right turn
    }
  } else if (isMovingStraight) {
    // Turn on green for moving straight
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 0));  // Green for moving straight
    }
  }
  strip.show();
}

void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }

    digitalWrite(GRIPPER_SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_SERVO, LOW);
    timer = millis() + 20;  // 20ms interval for servo
  }
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long _pulseDuration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  return (_pulseDuration * SOUND_SPEED) / 2;
}

// Function to check for obstacles and trigger avoidance
void checkObstacle() {
  _distance_cm = measureDistance();

  if (_distance_cm > 0 && _distance_cm < 10) {
    avoidObstacle();
  }
}

void avoidObstacle() {
  motorStop();
  delay(DURATION_DELAY_AVOID_OBSTACLE);

  motorRotate90DegreesRight();  // Turn 90° to the right 
  motorDrive(MAX_SPEED, MAX_SPEED);
  delay(DURATION_DELAY_AVOID_OBSTACLE);  // Drive forwards

  motorRotate90DegreesLeft();  // Turn 90° to the left
  motorDrive(MAX_SPEED, MAX_SPEED);
  delay(DURATION_MOVEMENT_FORWARDS_AVOID_OBSTACLE);  // Surpass the obstacle

  motorRotate90DegreesLeft();  // Turn 90° to the left
  motorDrive(MAX_SPEED, MAX_SPEED);
  delay(DURATION_DELAY_AVOID_OBSTACLE);  // Drive back towards original path

  motorRotate90DegreesRight();  // Turn 90° to the right (back on original path)

  // Measure distance again
  _distance_cm = measureDistance();
}

void followLine() {
  int sensorReadings[SENSOR_COUNT];
  int sum = 0;
  bool allBlack = true;  // Assume all sensors detect black initially

  // Read sensor values and compute the sum for averaging
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorReadings[i] = analogRead(_lineSensors[i]);
    sum += sensorReadings[i];

    // Calibrate the sensor values (use the previous min/max calibration)
    if (sensorReadings[i] < _sensorMin[i]) {
      _sensorMin[i] = sensorReadings[i];
    }
    if (sensorReadings[i] > _sensorMax[i]) {
      _sensorMax[i] = sensorReadings[i];
    }

    // If at least one sensor is NOT black, set allBlack to false
    if (sensorReadings[i] < _sensorMax[i] - 100) {
      allBlack = false;
    }
  }

  if (allBlack) {
    if (_blackStartTime == 0) {
      _blackStartTime = millis();                  // Start the timer
    } else if (millis() - _blackStartTime >= 150)  // Check if 150ms have passed
    {
      motorStop();                         // Stop motors
      setDirectionLights(false, false, false);  // Turn off all direction lights 
      gripper(GRIPPER_OPEN);
      _isStopping = true;
      return;  // Stop execution
    }
  } else {
    _blackStartTime = 0;  // Reset timer if not all sensors are black
    _isStopping = false;
  }

  if (_isStopping) {
    return;  // If it's stopping, don't process movement
  }

  // Calculate the average sensor value
  int averageSensorValue = sum / SENSOR_COUNT;
  int deadzoneLow = averageSensorValue - 50;
  int deadzoneHigh = averageSensorValue + 50;

  // Determine the current direction based on sensor readings
  if (sensorReadings[6] >= deadzoneHigh && sensorReadings[7] >= deadzoneHigh) {
    motorDrive(MAX_SPEED, -120);             // Turn left
    setDirectionLights(true, false, false);  // Left turn lights
  } else if (sensorReadings[3] >= deadzoneHigh && sensorReadings[4] >= deadzoneHigh) {
    motorDrive(STEADY_SPEED, STEADY_SPEED);  // Go straight
    setDirectionLights(false, false, true);  // Straight movement lights
  } else if (sensorReadings[0] >= deadzoneHigh && sensorReadings[1] >= deadzoneHigh) {
    motorDrive(-120, MAX_SPEED);             // Turn right
    setDirectionLights(false, true, false);  // Right turn lights
  } else if (sensorReadings[4] >= deadzoneHigh && sensorReadings[5] >= deadzoneHigh) {
    motorDrive(STEADY_SPEED, 165);           // Slightly to the right
    setDirectionLights(false, true, false);  // Right turn lights
  } else if (sensorReadings[5] >= deadzoneHigh && sensorReadings[6] >= deadzoneHigh) {
    motorDrive(STEADY_SPEED, 0);            // More to the right
    setDirectionLights(false, true, false);  // Right turn lights
  } else if (sensorReadings[2] >= deadzoneHigh && sensorReadings[3] >= deadzoneHigh) {
    motorDrive(165, STEADY_SPEED);           // Slightly to the left
    setDirectionLights(true, false, false);  // Left turn lights
  } else if (sensorReadings[1] >= deadzoneHigh && sensorReadings[2] >= deadzoneHigh) {
    motorDrive(0, STEADY_SPEED);            // More to the left
    setDirectionLights(true, false, false);  // Left turn lights
  } else if (sum < deadzoneLow * SENSOR_COUNT) {
    motorDrive(-255, 255);  // Search for the line
  }

  delay(50);  // Add a small delay to smooth the motor control
}
