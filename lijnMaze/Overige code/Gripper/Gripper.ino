// Library initialiseren
#include <Adafruit_NeoPixel.h>

// NEOPIXEL
#define PIN 2       // Data pin connected to DIN
#define NUMPIXELS 4  // neopixel led count
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

// Motor pins
#define MOTORA1 11  // Richting & PWM
#define MOTORA2 10  // Richting & PWM
#define SENSOR_R1 4 // MH-sensor links
#define MOTORB1 6   // Richting & PWM
#define MOTORB2 5   // Richting & PWM
#define SENSOR_R2 8 // MH-sensor rechts

// Ultrasonic sensor pin
#define TRIGPIN 9
#define ECHOPIN 3
float _DURATION, _DISTANCE;

// Button pin
#define BUTTONPIN2 2
bool buttonState = 0;

// Gripper pin
#define GRIPPER 3

// Lijnsensor pin
#define _NUM_SENSORS 8
const int LineSensor[_NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// SETUP
void setup() {
    Serial.begin(9600);

    //motor setup
    pinMode(MOTORA1, OUTPUT);
    pinMode(MOTORA2, OUTPUT);
    pinMode(MOTORB1, OUTPUT);
    pinMode(MOTORB2, OUTPUT);

    pinMode(SENSOR_R1, INPUT_PULLUP);
    pinMode(SENSOR_R2, INPUT_PULLUP);

    // knop setup
    pinMode(BUTTONPIN2, INPUT);
    //IFR Setup
    pinMode(TRIGPIN, OUTPUT);  
    pinMode(ECHOPIN, INPUT); 
    //gripper setup
    pinMode(GRIPPER, OUTPUT); 

    //Pixel setup
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Zet alle sensors als input
    for (int i = 0; i < _NUM_SENSORS; i++) {
        pinMode(LineSensor[i], INPUT);
    }
}

// LOOP
void loop() {
    buttonState = digitalRead(BUTTONPIN2);

    if (buttonState == LOW) {
        unsigned long startTime = millis();  // Start time for the sequence

        while (millis() - startTime < 7500) {
            unsigned long elapsedTime = millis() - startTime;
            gripper(0);
            if (elapsedTime < 100) {
                gripperClosed();
            } else if (elapsedTime < 1000) {
                gripperOpen();
            } else if (elapsedTime < 2500) {
                forward();
            } else if (elapsedTime < 3000) {
                gripperClosed();
            } else if (elapsedTime < 6000) {
                forward();
            } else if (elapsedTime < 7000) {
                stopMotorControl();
                gripperOpen();
            } else if (elapsedTime < 7500) {
                backward();
            }
        }
    }
    stopMotorControl();

}

void motorControl(int motorA1, int motorA2, int motorB1, int motorB2) {
    analogWrite(MOTORA1, motorA1);
    analogWrite(MOTORA2, motorA2);
    analogWrite(MOTORB1, motorB1);
    analogWrite(MOTORB2, motorB2);
}

void forward() {
    regularLight();
    motorControl(0, 215, 0, 215);
}

void backward() {
    brakeLight();
    motorControl(220, 0, 220, 0);
}

void right45() {  
    unsigned long startTime = millis();
    while (millis() - startTime < 237) { 
        motorControl(0, 255, 255, 0);
        blinkerRight();
    }
    stopMotorControl(); // Stop the motor after the turn
}

void right90() {
    unsigned long startTime = millis();
    while (millis() - startTime < 450) { 
        motorControl(0, 255, 255, 0);
        blinkerRight();
    }
    stopMotorControl(); // Stop the motor after the turn
}

void left45() {
    blinkerLeft();
    unsigned long startTime = millis();
    motorControl(255, 0, 0, 255);
    while (millis() - startTime < 237) { // Adjust the duration as needed
    // Wait for the desired duration in milliseconds
    }
}

void left90() {
    blinkerLeft();
    unsigned long startTime = millis();
    motorControl(255, 0, 0, 255);
    while (millis() - startTime < 400) { // Adjust the duration as needed
        // Wait for the desired duration in milliseconds
    }
}

void stopMotorControl() {
    regularLight();
    motorControl(0, 0, 0, 0);
}

// Gripper initialiseren
void gripper(int pulse) {
    static unsigned long timer;
    static int lastPulse;
    if (millis() > timer) {
        if (pulse > 0) {
            lastPulse = pulse;
        } else {
            pulse = lastPulse;
        }
        digitalWrite(GRIPPER, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER, LOW);
        timer = millis() + 20; //20ms interval voor servo
    }
}

// Gripper open functie
void gripperOpen() {
    gripper(1800);
}

// Gripper closed functie
void gripperClosed() {
    gripper(850);
}

// Functie voor ultra sonic sensor
void ifrSensor() {
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
}

// Serial Monitor info ultra sonic
void ifrInformation() {
    _DURATION = pulseIn(ECHOPIN, HIGH);
    _DISTANCE = (_DURATION*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(_DISTANCE);
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 100) { // 100 millisecond delay
        lastMillis = millis();
    }
}

// Blinkers neopixel
void blinkers(int boven, int onder, bool active) {
    static unsigned long previousMillis = 0;
    static bool ledState = false;  // Ensure ledState is retained across calls
    unsigned long currentMillis = millis();

    if (active == true) {  // Alleen knipperen als "active" true is
        if (currentMillis - previousMillis >= 500) {  
            previousMillis = currentMillis;
            ledState = !ledState;  // Toggle state
        }
    } else {
        ledState = false;
        previousMillis = 0;  // LED blijft uit als knipperen stopt
    }

    if (ledState) {
        strip.setPixelColor(boven, strip.Color(255, 69, 0));  // Orange ON
        strip.setPixelColor(onder, strip.Color(255, 69, 0));  // Orange ON
    } else {
        strip.setPixelColor(boven, strip.Color(100, 0, 0));  // Red
        strip.setPixelColor(onder, strip.Color(100, 100, 100)); // White
    }

    strip.show();  // Update the strip to reflect changes
}

void blinkerLeft() {
    strip.clear();
    blinkers(0, 3, true);
    strip.setPixelColor(1, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.show();
}

void blinkerRight() {
    strip.clear();
    blinkers(1, 2, true);
    strip.setPixelColor(0, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}

void brakeLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
    }

void regularLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}