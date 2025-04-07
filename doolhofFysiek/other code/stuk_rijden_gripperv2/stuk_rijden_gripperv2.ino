#include <Adafruit_NeoPixel.h>

#define NeoLED 8
#define NUM_PIXELS 4
#define INTERVAL 1000UL

#define GRIPPER 7 
#define GRIPPER_OPEN 1750
#define GRIPPER_CLOSE 1090

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

const int Button1 = 4;
const int MotorL_1 = 9;
const int MotorL_2 = 10;
const int MotorR_1 = 5;
const int MotorR_2 = 6;
const int MotorCorrection = 10;

unsigned long startMillis = 0;
bool buttonPressed = false;
bool sequenceStarted = false;
bool gripperClosed = false;

void setup() {
    pinMode(GRIPPER, OUTPUT);
    pinMode(Button1, INPUT_PULLUP);
    pinMode(MotorL_1, OUTPUT);
    pinMode(MotorL_2, OUTPUT);
    pinMode(MotorR_1, OUTPUT);
    pinMode(MotorR_2, OUTPUT);

    strip.begin();
    strip.show();
    Serial.begin(9600);
}

void loop() {
    unsigned long currentMillis = millis();
    const int debounceDelay = 50;
    static unsigned long lastDebounceTime = 0;

    int buttonState = digitalRead(Button1);

    if (buttonState == LOW && !buttonPressed) {
        if (currentMillis - lastDebounceTime > debounceDelay) {
            buttonPressed = true;
            startMillis = currentMillis;
            sequenceStarted = true;
            lastDebounceTime = currentMillis;
            Serial.println("Button Pressed! Starting sequence...");
        }
    }

    if (buttonState == HIGH && buttonPressed) {
        buttonPressed = false;
        lastDebounceTime = currentMillis;
    }

    if (sequenceStarted) {
        unsigned long elapsed = currentMillis - startMillis;

        if (elapsed == 1000UL) moveServo(GRIPPER_OPEN);
        if (elapsed == 2000UL) moveServo(GRIPPER_CLOSE);
        if (elapsed == 3000UL) moveServo(GRIPPER_OPEN);
        if (elapsed == 6000UL) MoveForward(255);
        if (elapsed == 8000UL) { moveServo(GRIPPER_CLOSE); gripperClosed = true; }
        if (elapsed == 10000UL) TurnRightWithCurve(255, 50);
        if (elapsed == 13000UL) MotorsStop();
        if (elapsed == 14000UL) TurnLeftWithCurve(255, 50);
        if (elapsed == 16000UL) MotorsStop();
        if (elapsed == 18000UL) MoveBackwards(255);
        if (elapsed == 20000UL) { 
            moveServo(GRIPPER_OPEN); // Move this here so it gets the last pulse
            gripperClosed = false; 
            MotorsStop(); 
            sequenceStarted = false; 
        }
    } 
    else {
        BlinkStillLED();
    }

    if (gripperClosed && sequenceStarted) {
        moveServo(GRIPPER_CLOSE); // Only keep it closed while the sequence is running
    }
}

void moveServo(int pulse) {
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
        timer = millis() + 20; // 20ms interval for servo
    }
}

void TurnMotorLeft(int speed) {
    analogWrite(MotorL_1, speed);
    analogWrite(MotorL_2, 0);
}

void TurnMotorRight(int speed) {
    analogWrite(MotorR_1, max(speed - MotorCorrection, 0));
    analogWrite(MotorR_2, 0);
}

void TurnMotorReverseLeft(int speed) {
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, speed);
}

void TurnMotorReverseRight(int speed) {
    analogWrite(MotorR_1, 0);
    analogWrite(MotorR_2, max(speed - MotorCorrection, 0));
}

void MoveForward(int speed) {
    TurnMotorLeft(speed);
    TurnMotorRight(speed);
    FrontLED();
}

void MoveBackwards(int speed) {
    TurnMotorReverseLeft(speed);
    TurnMotorReverseRight(speed);
    BackLED();
}

void TurnLeftWithCurve(int speed, int angle) {
    TurnMotorRight(max(speed - angle, 0));
    TurnMotorLeft(speed);
    LeftLED();
}

void TurnRightWithCurve(int speed, int angle) {
    TurnMotorRight(speed);
    TurnMotorLeft(max(speed - angle, 0));
    RightLED();
}

void MotorsStop() {
    analogWrite(MotorR_1, 0);
    analogWrite(MotorR_2, 0);
    analogWrite(MotorL_1, 0);
    analogWrite(MotorL_2, 0);
    StillLED();
}

void FrontLED() {
    strip.setPixelColor(2, strip.Color(255, 255, 255));
    strip.setPixelColor(3, strip.Color(255, 255, 255));
    strip.show();
}

void LeftLED() {
    strip.setPixelColor(0, strip.Color(45, 200, 10));
    strip.setPixelColor(3, strip.Color(45, 200, 10));
    strip.show();
}

void RightLED() {
    strip.setPixelColor(1, strip.Color(45, 200, 10));
    strip.setPixelColor(2, strip.Color(45, 200, 10));
    strip.show();
}

void BackLED() {
    strip.setPixelColor(0, strip.Color(255, 255, 255));
    strip.setPixelColor(1, strip.Color(255, 255, 255));
    strip.show();
}

void StillLED() {
    for (int i = 0; i < 4; i++) {
        strip.setPixelColor(i, strip.Color(45, 200, 10));
    }
    strip.show();
}

void BlinkStillLED() {
    static unsigned long ledBlinkMillis = 0;
    static bool ledState = false;

    if (millis() - ledBlinkMillis >= 500) {
        ledBlinkMillis = millis();
        ledState = !ledState;

        if (ledState) { 
            StillLED(); 
        } else { 
            strip.clear(); 
            strip.show(); 
        }
    }
}
