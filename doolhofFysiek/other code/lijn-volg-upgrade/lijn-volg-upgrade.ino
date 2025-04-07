unsigned long previousMillis = 0UL;
unsigned long interval = 200UL;

// line sensor
int lineSensor0 = 0;
int lineSensor1 = 1;
int lineSensor2 = 2;
int lineSensor3 = 4;
int lineSensor4 = 3;
int lineSensor5 = 5;
int lineSensor6 = 6;
int lineSensor7 = 7;
int sensorTarget = 900;

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

unsigned long interval2 = 20UL;

// State management
enum State { IDLE, FORWARD, STOP1, BACKWARD, STOP2 };
State currentState = IDLE;
unsigned long stateStartTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(button1, INPUT_PULLUP); // Use internal pull-up resistor
}

void loop() {
  unsigned long currentTime = millis();
  

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval)
  {
    // cws1 = pulseIn(ws1, HIGH);
    // cws2 = pulseIn(ws2, HIGH);
    // Read sensor continuously
    // Serial.println(pulseIn(ws1, HIGH, interval) - pulseIn(ws2, HIGH, interval));
    // Serial.println(cws1 - cws2);
    
    
    // int bs1 = RoundOfNumber(analogRead(lineSensor0) + analogRead(lineSensor1));
    // int bs2 = RoundOfNumber(analogRead(lineSensor2) + analogRead(lineSensor3));
    // int bs3 = RoundOfNumber(analogRead(lineSensor4) + analogRead(lineSensor5));
    // int bs4 = RoundOfNumber(analogRead(lineSensor6) + analogRead(lineSensor7));

    // int bs1 = analogRead(lineSensor0) + analogRead(lineSensor1);
    // int bs2 = analogRead(lineSensor2) + analogRead(lineSensor3);
    // int bs3 = analogRead(lineSensor4) + analogRead(lineSensor5);
    // int bs4 = analogRead(lineSensor6) + analogRead(lineSensor7);


    int bs1 = analogRead(lineSensor0);
    int bs2 = analogRead(lineSensor3);
    int bs3 = analogRead(lineSensor4);
    int bs4 = analogRead(lineSensor7);

    Serial.println(bs1);
    Serial.println(bs2);
    Serial.println(bs3);
    Serial.println(bs4);

    // Serial.println(analogRead(lineSensor0));
    // Serial.println(analogRead(lineSensor1));
    // Serial.println(analogRead(lineSensor2));
    // Serial.println(analogRead(lineSensor3));
    // Serial.println(analogRead(lineSensor4));
    // Serial.println(analogRead(lineSensor5));
    // Serial.println(analogRead(lineSensor6));
    // Serial.println(analogRead(lineSensor7));

    // if ((bs2 + bs3) < (bs1 + bs4)) {
    //   Serial.println("Forward");
    // } else if((bs1 + bs2) > (bs3 + bs4)) {
    //   Serial.println("Left");
    // } else if((bs1 + bs2) < (bs3 + bs4)) {
    //   Serial.println("Right");
    // } else {
    //   Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHH!!");
    // }

    if ((bs2 > sensorTarget) || (bs3 > sensorTarget)) {
      // Serial.println(bs2);
      // Serial.println(bs3);
      Serial.println("Forward");
      MoveForward(200);
    } else if(bs1 > sensorTarget) {
      // Serial.println(bs1);
      Serial.println("Left");
      TurnLeftWithCurve(150, 180);
    } else if(bs4 > sensorTarget) {
      // Serial.println(bs4);
      Serial.println("Right");
      TurnRightWithCurve(150, 180);
    } else {
      Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHH!!");
    }

    Serial.println("///////////////////////////////");
  	previousMillis = currentMillis;
  }

  // Read button state
  int buttonState = digitalRead(button1);

  // Detect button press (transition from HIGH to LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    
  }

  lastButtonState = buttonState; // Update button state


}

// ###### fuctions ######

int RoundOfNumber(int number) {
  return ((number + 50) / 200) * 200 ;
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




