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
int MotorCorrection = 8;

// Button setup
int button1 = 2;
int lastButtonState = HIGH; // Track last button state
bool sequenceRunning = false; // Track if the movement sequence is active

int ws1 = 3;
int ws2 = 4;

void setup() {
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
}

void loop() {
  //unsigned long currentTime = millis();
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) { 
    // Read button state
    int buttonState = digitalRead(button1);

    // Detect button press (transition from HIGH to LOW)
    if (buttonState == LOW && lastButtonState == HIGH) {
      // calibration();
      followLine();
    }
    lastButtonState = buttonState; // Update button state
  
  previousMillis = currentMillis;
  }
}

// ###### fuctions ######

void followLine() { 
  unsigned long startTime = millis();
  unsigned long previousMillis = 0;
  const unsigned long interval = 1000; // Interval in milliseconds
  Serial.println("follow line");

  while (millis() - startTime < 20000) { // Run for 10 seconds
    for (int i = 0; i < 8; i++) {
      if (analogRead(lineSensors[i]) > sensorTarget) {
        // Serial.println(analogRead(lineSensors[i]));
        if (i > 3) {
          Serial.print("right ");
          Serial.println(lineFollowCorrection[i]);
          
          if (i = 3) {
            // TurnRight(255);
            TurnLeft(255);
          } else {
            TurnRightWithCurve(255, lineFollowCorrection[i]);
          }
        } else if (i < 3) {
          Serial.print("Left ");
          Serial.println(lineFollowCorrection[i]);
          
          TurnLeft(255);
          if (i = 7) {
            // TurnLeft(255);
            TurnRight(255);
          } else {
            TurnLeftWithCurve(255, lineFollowCorrection[i]);
          }
        }
      } else {
        MoveForward(255);
      }
    }
  }
  
  MotorsStop();
}

void calibration() {
  Serial.println("//////////");
  Serial.println("### Calibration Start ###");

  unsigned long startTime = millis();
  unsigned long previousMillis = 0;
  const unsigned long interval = 1000; // Interval in milliseconds

  while (millis() - startTime < 5000) { // Run for 5 seconds
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      for (int i = 0; i < 8; i++) {
        Serial.println(analogRead(lineSensors[i]));
      }
      Serial.println("//////////");
    }
    // Keep moving forward continuously
    MoveForward(255);
  }
  
  MotorsStop();
  Serial.println("### Calibration Complete ###");
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




