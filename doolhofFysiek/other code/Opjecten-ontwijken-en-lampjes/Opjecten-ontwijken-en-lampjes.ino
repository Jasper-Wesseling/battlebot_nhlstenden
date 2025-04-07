#include <Adafruit_NeoPixel.h>

#define NeoLED 8          // The single data pin for all NeoPixels
#define NUM_PIXELS 4      // Total number of NeoPixels (e.g., 4 LEDs)
#define DELAY_TIME 500    // Delay time between LED blinks

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

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
  
  // **Motor pins defined as OUTPUT**
  pinMode(MotorL_1, OUTPUT);
  pinMode(MotorL_2, OUTPUT);
  pinMode(MotorR_1, OUTPUT);
  pinMode(MotorR_2, OUTPUT);
  
  Serial.begin(9600); // Start serial monitor
  
  strip.begin();
  strip.show(); // **Ensure all LEDs start OFF**
}

void loop() {
  strip.clear();  // Turn off all LEDs
  
  long duration;
  int distance;

  // **Trigger the ultrasonic sensor correctly**
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time
  duration = pulseIn(echoPin, HIGH, 30000);  // **Fix: Add timeout to avoid hanging**
  
  // Convert to distance in cm
  distance = duration * 0.034 / 2;

  // Debugging: Print distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance <= shortDistance) {
    // Object detected: Stop, then turn left
    Serial.println("Object detected! Turning left...");
    
    // Stop motors first for better turning
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, 0);
    analogWrite(MotorR_1, 0);
    analogWrite(MotorR_2, 0);
    delay(200);  // Pause before turning

    // **Turn on Left LEDs**
    strip.setPixelColor(0, strip.Color(255, 165, 0)); // Orange
    strip.setPixelColor(3, strip.Color(255, 165, 0)); // Orange
    strip.show(); // **Fix: Update LED state before movement**

    // Turn left
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, 180); // **Increase power for better turning**
    analogWrite(MotorR_1, 180);
    analogWrite(MotorR_2, 0);
    delay(500); // Turn for 500ms
  } else {
    // Move forward
    Serial.println("No obstacle, moving forward...");
    
    // **Turn on Front LEDs**
    strip.setPixelColor(2, strip.Color(255, 255, 255)); // White (Front-left)
    strip.setPixelColor(3, strip.Color(255, 255, 255)); // White (Front-right)
    strip.show(); // **Fix: Update LED state before movement**

    // Move forward
    analogWrite(MotorL_1, 150);
    analogWrite(MotorL_2, 0); 
    analogWrite(MotorR_1, 150);
    analogWrite(MotorR_2, 0);
  }
}
