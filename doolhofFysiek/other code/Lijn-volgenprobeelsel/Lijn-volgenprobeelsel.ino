const int IR_PIN_ONE = A0;
const int IR_PIN_FOUR = A1;
const int IR_PIN_FIVE = A2;
const int IR_PIN_EIGHT = A3;

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_PIN_ONE, INPUT);
  pinMode(IR_PIN_FOUR, INPUT);
  pinMode(IR_PIN_FIVE, INPUT);
  pinMode(IR_PIN_EIGHT, INPUT);
}


void loop()
{
  // IR sensor readings and movement control based on sensor readings
  if(analogRead(IR_PIN_ONE) > IR_COLOR_BLACK)
  {
    moveForward(240,200);
  }
  if(analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK)
  {
    moveForward(240,220);
  }
  if(analogRead(IR_PIN_FIVE) > IR_COLOR_BLACK)
  {
    moveForward(220,240);
  }
  if(analogRead(IR_PIN_EIGHT) > IR_COLOR_BLACK)
  {
    moveForward(200,240);
  }
  if (analogRead(IR_PIN_ONE) > IR_COLOR_BLACK && analogRead(IR_PIN_FOUR) > IR_COLOR_BLACK && analogRead(IR_PIN_FIVE) > IR_COLOR_BLACK && analogRead(IR_PIN_EIGHT) > IR_COLOR_BLACK )
  {
    stopMoving();
    delay(250);
    moveGripper(100);
    exit(0);
  }
}
