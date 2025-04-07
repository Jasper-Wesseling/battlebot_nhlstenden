//-----------------------------Team Informatie------------------------------//
// RobotNummer: BB008
// Robot naam: BB-8
// Groep: IC-INF-1C (IDrunkDrive)
// Contributors: Dinand Rengers & Tim Kap
// COPYRIGHT: © 2025 | IDrunkDrive

//----------------BreadBord-Pin-Installatie + Variabelen---------------------//

//    ╔════════════════════════════════════════════════════╗
//    ║               ARDUINO PIN CONFIGURATION           ║
//    ╠════════════════════╦══════════════════════════════╣
//    ║      ANALOG PINS   ║        FUNCTION              ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║ A0                 ║ Sensor 1 - Infrared Line     ║
//    ║ A1                 ║ Sensor 2 - Infrared Line     ║
//    ║ A2                 ║ Sensor 3 - Infrared Line     ║
//    ║ A3                 ║ Sensor 4 - Infrared Line     ║
//    ║ A4                 ║ Sensor 5 - Infrared Line     ║
//    ║ A5                 ║ Sensor 6 - Infrared Line     ║
//    ║ A6                 ║ Sensor 7 - Infrared Line     ║
//    ║ A7                 ║ Sensor 8 - Infrared Line     ║
//    ╠════════════════════╬══════════════════════════════╣
//    ╠════════════════════╬══════════════════════════════╣
//    ║   MOTOR CONTROL    ║                              ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║ 5                  ║ B2_MOTOR_PIN                 ║
//    ║ 6                  ║ B1_MOTOR_PIN                 ║
//    ║ 10                 ║ A2_MOTOR_PIN                 ║
//    ║ 11                 ║ A1_MOTOR_PIN                 ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║  LEDS & SERVOS     ║                              ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║ 3                  ║ GRIPPER_PIN - Servo Gripper  ║
//    ║ 2                  ║ PIXEL_PIN - Corner Lamps     ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║ ULTRASONIC SENSOR  ║                              ║
//    ╠════════════════════╬══════════════════════════════╣
//    ║ 7                  ║ ECHO_PIN                     ║
//    ║ 4                  ║ TRIGGER_PIN                  ║
//    ╚════════════════════╩══════════════════════════════╝

// LIBRARIES
#include <Adafruit_NeoPixel.h>

// MOTOR //
#define MOTOR_A1 11
#define MOTOR_A2 10
#define MOTOR_B1 6
#define MOTOR_B2 5

// GRIPPER //
#define GRIPPER 3
#define ROTATION_1 2

// LIJNSENSOR //
#define NUMSENSORS 8
const int sensorPins[NUMSENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ECHO SONIC SENSOR //
#define TRIGGERPIN 4
#define ECHOPIN 7

// BUTTON
#define BUTTON1 2

// NEOPIXEL //
#define PIN 8
#define NUMPIXELS 4
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

// MOTOR SNELHEDEN
#define FULLSPEED 255
#define STEADY_SPEED 214
#define SLOW_SPEED 180
#define SLOWER_SPEED 60
#define SLOWEST_SPEED 10
#define SEARCH_SPEED 50

// ULTRA SENSOR VARIABELEN
float SHORTDISTANCE = 14.00;
float _PULSEDURATION, DISTANCEINCM;

bool _ISOBJECTDETECTED = false;
bool objectDetected = false;

//-----------------------------SETUP------------------------------//
void setup() { 
  pinMode(MOTOR_A1, OUTPUT); 
  pinMode(MOTOR_A2, OUTPUT); 
  pinMode(MOTOR_B1, OUTPUT); 
  pinMode(MOTOR_B2, OUTPUT); 
  pinMode(ROTATION_1, INPUT);

  pinMode(GRIPPER, OUTPUT);
  gripperOpen();

  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  strip.begin();
  strip.clear();
  strip.show();

  Serial.begin(9600);
} 

//-----------------------------LOOP------------------------------//
void loop()
{
  followLine();
}

//-----------------------------FUNCTIES------------------------------//

// MOTOR FUNCTIES
void drive(int leftBackwards, int leftForward, int rightBackwards, int rightForward) { 
  analogWrite(MOTOR_A1, leftBackwards); 
  analogWrite(MOTOR_A2, leftForward); 
  analogWrite(MOTOR_B1, rightBackwards); 
  analogWrite(MOTOR_B2, rightForward); 
} 

// MOTORSTOPPEN FUNCTIE
void stop() { 
  analogWrite(MOTOR_A1, 0); 
  analogWrite(MOTOR_A2, 0); 
  analogWrite(MOTOR_B1, 0); 
  analogWrite(MOTOR_B2, 0); 
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
        timer = millis() + 20;
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

// START FUNCTIE
void start() {
    // Blauwe lichten
    waitLights();

    // Gripper openen
    gripperOpen();
    delay(500);

    // Rij een stuk naar voren
    drive(0, 200, 0, 185);
    
    // Wacht totdat de robot op de lijn komt (sensor 3 en 4)
    while (analogRead(sensorPins[3]) < 800 && analogRead(sensorPins[4]) < 800) {
        delay(50);  // Check elke 50ms
    }
    
    stop();  // Stop zodra de lijn is gedetecteerd
    
    // Wacht totdat een object wordt gedetecteerd via de ultrasone sensor
    Serial.println("Wachten op object...");
    while (!checkForObject()) {
        delay(500);  // Check elke 500ms
    }
  
    // Zodra een object wordt gedetecteerd, rijd naar voren en pak de pion op
    drive(0, 190, 0, 180); 
    delay(400);
    stop();

    gripperClosed();
    delay(200);
}

// LIJNVOLG LOGICA
// Functie om objecten te detecteren via ultrasone sensor
bool checkForObject() {
    digitalWrite(TRIGGERPIN, LOW);  
    delayMicroseconds(2);  
    digitalWrite(TRIGGERPIN, HIGH);  
    delayMicroseconds(10);  
    digitalWrite(TRIGGERPIN, LOW);  
    _PULSEDURATION = pulseIn(ECHOPIN, HIGH);
    DISTANCEINCM = _PULSEDURATION * 0.0344 / 2;
  
    // Debugging: print de gemeten afstand
    Serial.print("Afstand: ");
    Serial.println(DISTANCEINCM);
  
    if (DISTANCEINCM <= SHORTDISTANCE) {
      Serial.println("Object gedetecteerd!");
      objectDetected = true;
      return true;
    } else {
      objectDetected = false;
      return false;
    }
}

// FOLLOW LINE FUNCTIE
void followLine() {
    int sensorReadings[NUMSENSORS];
    int sum = 0;
  
    for (int i = 0; i < NUMSENSORS; i++) {
      sensorReadings[i] = analogRead(sensorPins[i]);
      sum += sensorReadings[i];
    }
  
    int avg = sum / NUMSENSORS;
    int deadzonelow = avg - 50;
    int deadzonehigh = avg + 50;
  
    // Controleer of zwarte box is detecteerd
    if (detectBlackBox()) {
      stop();
      gripperOpen();
      drive(230, 0, 0, 220);
      delay(600);
      stop();
    }
  
    // Controleer op objecten
    if (checkForObject()) {
      drive(230, 0, 0, 220);
      delay(500);
      stop();
      objectDetected = false;
      return;
    }
  
    // Lijnvolg logica
    if (sensorReadings[0] >= deadzonehigh || sensorReadings[1] >= deadzonehigh) {
      blinkRight();
      drive(190, 0, 0, 230);
      while (sensorReadings[3] >= deadzonehigh && sensorReadings[4] >= deadzonehigh) {
        return;
      }
    }
  
    if (sensorReadings[6] >= deadzonehigh || sensorReadings[7] >= deadzonehigh) {
      blinkLeft();
      drive(0, 230, 190, 0);
      while (sensorReadings[3] >= deadzonehigh && sensorReadings[4] >= deadzonehigh) {
        return;
      }
    }
  
    else if (sensorReadings[3] >= deadzonehigh && sensorReadings[4] >= deadzonehigh) {
      activeLights();
      drive(0, 230, 0, 220);
    }
  
    else if (sensorReadings[5] >= deadzonehigh) {
      drive(0, 220, 0, 230);
    }
  
    else if (sensorReadings[2] >= deadzonehigh) {
      drive(0, 240, 0, 220);
    }
  
    else if (sum < deadzonehigh * NUMSENSORS) {
      drive(230, 0, 0, 220);
    }
}

// Functie voor het detecteren van de BLACKBOX
bool detectBlackBox() {
    int sensorReadings[NUMSENSORS];
    int sum = 0;
  
    // Lees alle lijnsensoren
    for (int i = 0; i < NUMSENSORS; i++) {
      sensorReadings[i] = analogRead(sensorPins[i]);
      sum += sensorReadings[i];
    }
  
    // Controleer of alle sensoren een lage waarde hebben (indicatie van zwart)
    if (sum < 200) {  // Pas de drempelwaarde aan indien nodig
      // Controleer of de zwarte box breed genoeg is
      if (sensorReadings[3] < 800 && sensorReadings[4] < 800) {  // Sensor 3 en 4 moeten ook zwart detecteren
        Serial.println("Zwarte box gedetecteerd!");
        return true;
      }
    }
    return false;
}

// NEOPIXEL LED Functies
void activeLights() {
  strip.clear();
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.setPixelColor(1, strip.Color(0, 255, 0));
  strip.setPixelColor(2, strip.Color(0, 255, 0));
  strip.setPixelColor(3, strip.Color(0, 255, 0));
  strip.show();
}

void waitLights() {
  strip.clear();
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.setPixelColor(1, strip.Color(0, 0, 255));
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.show();
}

void blinkLeft() {
  strip.clear();
  strip.setPixelColor(1, strip.Color(255, 0, 0));
  strip.setPixelColor(2, strip.Color(255, 0, 0));
  strip.show();
}

void blinkRight() {
  strip.clear();
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.setPixelColor(3, strip.Color(255, 0, 0));
  strip.show();
}
