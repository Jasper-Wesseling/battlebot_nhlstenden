// Motor setup
int MotorL_1 = 9;
int MotorL_2 = 10;
int MotorR_1 = 5;
int MotorR_2 = 6;

int SensorL = 3;  // Left motor speed sensor
int SensorR = 4;  // Right motor speed sensor

int Trigpin = 7;  // Ultrasonic sensor trigger pin
int Echopin = 8;  // Ultrasonic sensor echo pin

int baseSpeed = 200;   // Base speed for both motors
int speedCorrection = 10; // Speed adjustment step
int threshold = 10;  // Allowed difference before correction

void setup() {
  pinMode(MotorL_1, OUTPUT);
  pinMode(MotorL_2, OUTPUT);
  pinMode(MotorR_1, OUTPUT);
  pinMode(MotorR_2, OUTPUT);

  pinMode(SensorL, INPUT);
  pinMode(SensorR, INPUT);
  
  pinMode(Trigpin, OUTPUT);
  pinMode(Echopin, INPUT);
  
  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Read speed feedback from sensors
  int speedL = pulseIn(SensorL, HIGH, 10000); // Timeout after 10ms
  int speedR = pulseIn(SensorR, HIGH, 10000);

  int motorL_Speed = baseSpeed;
  int motorR_Speed = baseSpeed;

  // Speed correction logic
  if (speedL > speedR + threshold) {
    motorL_Speed -= speedCorrection; // Slow down left motor
  } else if (speedR > speedL + threshold) {
    motorR_Speed -= speedCorrection; // Slow down right motor
  }

  // Ensure values are within 0-255 range
  motorL_Speed = constrain(motorL_Speed, 0, 255);
  motorR_Speed = constrain(motorR_Speed, 0, 255);

  // Apply corrected speeds to motors
  analogWrite(MotorL_1, motorL_Speed);
  analogWrite(MotorL_2, 0);
  analogWrite(MotorR_1, motorR_Speed);
  analogWrite(MotorR_2, 0);

  // Print motor speeds for debugging
  Serial.print("L: ");
  Serial.print(motorL_Speed);
  Serial.print(" R: ");
  Serial.println(motorR_Speed);

  // Ultrasonic sensor measurement
  long duration;
  float distance;

  digitalWrite(Trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigpin, LOW);

  duration = pulseIn(Echopin, HIGH, 30000); // 30ms timeout to avoid blocking

  distance = microsecondsToCentimeters(duration);

  Serial.println(distance); // Print raw number instead of formatted text

  delay(50); // Reduce delay for smoother operation
}

// Convert time to distance
float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}
