//-----------------------------Team Informatie------------------------------//
  // RobotNummer: BB008
  // Robot naam: BistDuFreaky?
  // Groep: IC-INF-1C (IDrunkDrive)
  // Contributors: Dinand Rengers & Tim Kap

  // COPYRIGHT: © 2025 | IDrunkDrive

//----------------BreadBord-Pin-Installatie + Variabelen---------------------//

  // LIBRARIES //
    #include <Adafruit_NeoPixel.h>

  // BESTURING //
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
    int sensorMin[NUMSENSORS];
    int sensorMax[NUMSENSORS];

    // ECHO SONIC SENSOR //
    #define TRIGPIN 4
    #define ECHOPIN 7

    // BLUETOOTH //
    #define BLUETOOTH_RX 0

    // NEOPIXEL //
    #define PIN 2
    #define NUMPIXELS 4
    Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

    // DEBUGGING //
      //#define SENSORVALUE

    // OVERIGE VARIABELEN //

      // MOTOR SNELHEDEN
      #define FULLSPEED 255
      #define STEADY_SPEED 214
      #define SLOW_SPEED 180
      #define SLOWER_SPEED 60
      #define SLOWEST_SPEED 10
      #define SEARCH_SPEED 50

      // LIJNVOLG LOGICA
      int _lastDirection = 0;  // 0 = recht, -1 = links, 1 = rechts
      const float rightTurnSpeedFactor = 0.9; // Verlaag rechter motor met 10%

//-----------------------------SETUP------------------------------//
void setup() { 
  pinMode(MOTOR_A1, OUTPUT); 
  pinMode(MOTOR_A2, OUTPUT); 
  pinMode(MOTOR_B1, OUTPUT); 
  pinMode(MOTOR_B2, OUTPUT); 
  pinMode(ROTATION_1, INPUT);
  pinMode(GRIPPER, OUTPUT);

  Serial.begin(9600); 
} 

//-----------------------------LOOP------------------------------//
void loop() {
  followLine();
}

//-----------------------------FUNCTIES------------------------------//

  // MOTOR FUNCTIES
    // RIJVOORUIT FUNCTIE
      void drive(int left, int right) { 
        analogWrite(MOTOR_A1, max(0, -left)); 
        analogWrite(MOTOR_A2, max(0, left)); 
        analogWrite(MOTOR_B1, max(0, -right)); 
        analogWrite(MOTOR_B2, max(0, right)); 
      }

    // MOTORSTOPPEN FUNCTIE
      void stop() { 
        analogWrite(MOTOR_A1, 0); 
        analogWrite(MOTOR_A2, 0); 
        analogWrite(MOTOR_B1, 0); 
        analogWrite(MOTOR_B2, 0); 
      } 

  // GRIPPER FUNCTIES
    // GRIPPER INITIALISATIE
      void gripper(int pulse) {
          gripper(0);
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

    // GRIPPER OPEN
      void gripperOpen() {
          gripper(1800);
      }

    // GRIPPER DICHT
      void gripperClosed() {
          gripper(1050);
      }

  // LIJNVOLGER FUNCTIES
    // LIJN VOLGEN
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

        int currentDirection = 0;  // Houd de huidige richting bij

        // Bepaal de huidige toestand
        if (sensorReadings[6] >= deadzonehigh && sensorReadings[7] >= deadzonehigh) { 
            currentDirection = -1;  // Rechts veel lijn -> draai links
            drive(FULLSPEED, -120);  // Draai links
        }
        else if (sensorReadings[3] >= deadzonehigh && sensorReadings[4] >= deadzonehigh) { 
          currentDirection = 0;  // Midden op lijn -> rechtdoor
          drive(STEADY_SPEED, STEADY_SPEED);  // Rechtdoor
        }
        else if (sensorReadings[0] >= deadzonehigh && sensorReadings[1] >= deadzonehigh) { 
            currentDirection = 1;  // Links veel lijn -> draai rechts
            drive(-120, FULLSPEED);  // Draai rechts
        }
        else if (sensorReadings[4] >= deadzonehigh && sensorReadings[5] >= deadzonehigh) { 
            currentDirection = 3;  // Iets rechts -> stuur beetje bij
            drive(STEADY_SPEED, 165);  // Iets naar rechts
        }
        else if (sensorReadings[5] >= deadzonehigh && sensorReadings[6] >= deadzonehigh) { 
            currentDirection = 4;  // Meer rechts -> stuur sterker bij
            drive(STEADY_SPEED, 35);  // Meer naar rechts
        }
        else if (sensorReadings[2] >= deadzonehigh && sensorReadings[3] >= deadzonehigh) { 
            currentDirection = 5;  // Iets links -> stuur beetje bij
            drive(165, STEADY_SPEED);  // Iets naar links
        }
        else if (sensorReadings[1] >= deadzonehigh && sensorReadings[2] >= deadzonehigh) { 
            currentDirection = 6;  // Meer links -> stuur sterker bij
            drive(35, STEADY_SPEED);  // Meer naar links
        }
        else if (sum < deadzonelow * NUMSENSORS) { 
            currentDirection = 7;  // Geen lijn -> draai zoeken
            drive(-255, 255);  // Zoeken naar lijn
        }

        // Als de richting veranderd is, print dan de nieuwe status
        if (currentDirection != _lastDirection) {
          _lastDirection = currentDirection;  // Werk de laatste richting bij

          switch (currentDirection) {
            case 0:
              Serial.println("Midden op lijn -> rechtdoor!");
              break;
            case 1:
              Serial.println("Links veel lijn -> draai rechts!");
              break;
            case -1:
              Serial.println("Rechts veel lijn -> draai links!");
              break;
            case 3:
              Serial.println("Iets rechts -> stuur beetje bij!");
              break;
            case 4:
              Serial.println("Meer rechts -> stuur sterker bij!");
              break;
            case 5:
              Serial.println("Iets links -> stuur beetje bij!");
              break;
            case 6:
              Serial.println("Meer links -> stuur sterker bij!");
              break;
            case 7:
              Serial.println("Geen lijn -> draai zoeken...");
              break;
            default:
              break;
          }
        }
        delay(50);
      }