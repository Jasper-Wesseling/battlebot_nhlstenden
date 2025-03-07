// Motor setup
int MotorL_1 = 9;
int MotorL_2 = 10;
int MotorR_1 = 5;
int MotorR_2 = 6;

// Ultrasonic sensor setup
const int trigPin = 13;
const int echoPin = 12;

// Detection threshold (10 cm)
const int shortDistance = 10;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600); // Start serial monitor
}

void loop() {
  long duration;
  int distance;

  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time
  duration = pulseIn(echoPin, HIGH);

  // Convert to distance in cm
  distance = duration * 0.034 / 2;

  // Debugging: Print distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance <= shortDistance) {
    // Object detected: Stop, then turn left
    Serial.println("Object detected! Turning left...");
    
    // Stop first for better turning
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, 0);
    analogWrite(MotorR_1, 0);
    analogWrite(MotorR_2, 0);
    delay(200);  // Pause before turning

    // Turn left
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, 150); // Reverse left motor
    analogWrite(MotorR_1, 150);
    analogWrite(MotorR_2, 0);
    delay(500); // Turn for 500ms
  } else {
 
    // Move forward
    Serial.println("No obstacle, moving forward...");
    analogWrite(MotorL_1, 200);
    analogWrite(MotorL_2, 0);
    analogWrite(MotorR_1, 200);
    analogWrite(MotorR_2, 0);
  }

  delay(100); // Small delay for stability
}
