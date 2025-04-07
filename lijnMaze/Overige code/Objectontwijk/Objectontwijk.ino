// BistDuFreaky, Tim & Dinand

// Include Arduino library
#include <Adafruit_NeoPixel.h>

// Debugging
#define DEBUG
//#define DEBUG_SENSOR_VALUE
//#define DEBUG_SENSOR

// Define NEOPIXEL 
#define NEOPIXEL_PIN 8
#define PXNUM 3

// Define THRESHOLD
#define THRESHOLD 915

Adafruit_NeoPixel pixels(PXNUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Geconstateerde HC-05 RX & TX pins
const int BLUETOOTH_RX = 1;
// Geconstateerde HC-SR04 sensor pins
const int TRIGGERPIN = 4;
const int ECHOPIN = 7;
// Geconstateerde motor pins
const int MOTORL_A1 = 6;   // Pin voor MotorA1, Links
const int MOTORL_A2 = 5;   // Pin voor MotorA2, Links
const int MOTORR_B1 = 10;  // Pin voor MotorB1, Rechts
const int MOTORR_B2 = 9;   // Pin voor MotorB2, Rechts

// Float variabelen aangeven
float SHORTDISTANCE = 12.00;
float _PULSEDURATION, DISTANCEINCM;

int sensorValues[8];  // Array om sensorwaarden op te slaan
int binaryValues[8];  // Hier slaan we de 0/1 waarden op

// Functie om motoren intergers te veranderen naar de waarden die ingevuld is in andere functies
void MotorRijden(int leftA1, int leftA2, int rightB1, int rightB2) {
    analogWrite(MOTORL_A1, leftA1);
    analogWrite(MOTORL_A2, leftA2);
    analogWrite(MOTORR_B1, rightB1);
    analogWrite(MOTORR_B2, rightB2);
}

// Functie voor motor vooruit
void MotorVooruitRijden() {
    MotorRijden(0, 255, 0, 245);
    pixels.clear();
}

// Functie voor motor achteruit
void MotorAchteruitRijden() {
    MotorRijden(255, 0, 245, 0);
    pixels.clear();
    pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Achterste linker LED
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Achterste rechter LED
    pixels.show();
}

// Functie voor motor stoppen
void MotorStoppen() {
    MotorRijden(0, 0, 0, 0);
    pixels.clear();
    pixels.show();
}

void MotorLinks() {
  MotorRijden(0, 0, 0, 255);
    pixels.clear();
    pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // Achterste linker LED
    pixels.show();
}

void MotorRechts() {
    MotorRijden(0, 255, 0, 0);
    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(0, 255, 0)); // Achterste rechter LED
    pixels.show();
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // Setup neopixels
  pixels.clear();
  pixels.begin(); // Initialiseer de Neopixel strip
  pixels.show();  // Zorg dat alle LED's uitgaan bij opstarten

  // Set pinmode bluetooth
  pinMode(BLUETOOTH_RX, OUTPUT);
  delay(1000);

  // Set pinmode neopixels
  pinMode(NEOPIXEL_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN, HIGH);

  pinMode(MOTORL_A1, OUTPUT);
  pinMode(MOTORL_A2, OUTPUT);
  pinMode(MOTORR_B1, OUTPUT);
  pinMode(MOTORR_B2, OUTPUT);

  digitalWrite(MOTORL_A1, HIGH);
  digitalWrite(MOTORL_A2, HIGH);
  digitalWrite(MOTORR_B1, HIGH);
  digitalWrite(MOTORR_B2, HIGH);

  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

}

void loop() {
  digitalWrite(TRIGGERPIN, HIGH);
  delay(10);
  digitalWrite(TRIGGERPIN, LOW);

  // Berekeningen float variabelen
  _PULSEDURATION = pulseIn(ECHOPIN, HIGH);
  DISTANCEINCM = 0.017 * _PULSEDURATION;

  // Print in Serial Monitor:
  #ifdef DEBUG
  Serial.print("Distance ");
  Serial.print(DISTANCEINCM);
  Serial.println(" cm ");
  delay(100);
  #endif

  // Als de gedectecteerde distance kleiner is dan de maximale short distance
  if (DISTANCEINCM <= SHORTDISTANCE) {
    #ifdef DEBUG
    Serial.println("Object gedecteerd! Achteruit met die handel!"); 
    delay(10);
    #endif
    MotorAchteruitRijden();
    delay(750);
    pixels.clear();
    delay(2);
    MotorRijden(0, 100, 0, 255);
    delay(500);
    MotorRijden(0, 255, 0, 100);
    delay(1500);
    MotorRijden(0, 100, 0, 255);
    delay(100);
  }
  else {
    MotorVooruitRijden();
  }
}