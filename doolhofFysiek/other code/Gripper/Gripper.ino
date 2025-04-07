#include <Servo.h>

const int Button1 = 4;  // Button pin
const int Gripper = 7;  // Servo signal pin

Servo gripServo; // Create a Servo object

void setup() {
  pinMode(Gripper, OUTPUT);
  pinMode(Button1, INPUT_PULLUP); // Enable internal pull-up for the button
  gripServo.attach(Gripper); // Attach the servo to the pin
}

void loop() {
  int buttonState = digitalRead(Button1);

  // If button is pressed, close the grip
  if (buttonState == LOW) { 
    MoveGrip(55);
  } else {
    MoveGrip(115);
  }
}

void MoveGrip(int angle) {
  gripServo.write(angle); // Move the servo to the desired angle
}
