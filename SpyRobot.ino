#include <IRremote.h>

#define LEFT_OPTICAL_PIN 2
#define RIGHT_OPTICAL_PIN 3
#define RECV_PIN 4
#define LEFT_MOTOR 5
#define RIGHT_MOTOR 6

#define CONTROL_MINUS 0xFFA857
#define CONTROL_PLUS 0xFF906F
#define CONTROL_OFF 0xFFA25D
#define CONTROL_ON 0xFF629D
#define CONTROL_REPEAT 0xFFFFFFFF

#define CONTROL_4 0xFF10EF
#define CONTROL_5 0xFF38C7
#define CONTROL_7 0xFF42BD
#define CONTROL_8 0xFF4AB5

#define OPERATION_TIME_MS 100

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long lastIrCommand = 0;
boolean leftMotorState = 0;
boolean rightMotorState = 0;
unsigned long timestamp = 0;
byte leftMotorHighValue = 255;
byte rightMotorHighValue = 255;
unsigned long clock100;
unsigned long clock1000;
int leftOpticalCounter = 0;
boolean lastLeftOpticalValue = LOW;
int rightOpticalCounter = 0;
boolean lastRightOpticalValue = LOW;

void MotorsControl(byte leftValue, byte rightValue)
{
  //Serial.println(__FUNCTION__);
  leftMotorState = leftValue;
  rightMotorState = rightValue;
}

void Left()
{
  //Serial.println(__FUNCTION__);
  leftMotorState = 0;
  rightMotorState = rightMotorHighValue;
}

void Right()
{
  //Serial.println(__FUNCTION__);
  leftMotorState = leftMotorHighValue;
  rightMotorState = 0;
}

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  analogWrite(LEFT_MOTOR, leftMotorState);
  analogWrite(RIGHT_MOTOR, rightMotorState);
  pinMode(LEFT_OPTICAL_PIN, INPUT);
  pinMode(RIGHT_OPTICAL_PIN, INPUT);
  clock100 = millis();
  clock1000 = clock100;
}

void loop() {
  unsigned long irCommand = 0;
  if (irrecv.decode(&results)) {
    //Serial.println(results.value, HEX);
    irCommand = results.value;
    if (irCommand != CONTROL_REPEAT && irCommand > 0)
    {
      lastIrCommand = irCommand;
    }
    irrecv.resume(); // Receive the next value
  }

  boolean knownCommand = false;
  switch (irCommand)
  {
    case CONTROL_PLUS:
      Left();
      knownCommand = true;
      break;
    case CONTROL_MINUS:
      Right();
      knownCommand = true;
      break;
    case CONTROL_OFF:
      MotorsControl(0, 0);
      knownCommand = true;
      break;
    case CONTROL_ON:
      MotorsControl(leftMotorHighValue, rightMotorHighValue);
      knownCommand = true;
      break;
    case CONTROL_REPEAT:
      //Serial.println("CONTROL_REPEAT");
      knownCommand = true;
      break;
    case CONTROL_4:
      Serial.print("leftMotorHighValue: "); Serial.print(leftMotorHighValue);
      Serial.print(" rightMotorHighValue: "); Serial.println(rightMotorHighValue);
      leftOpticalCounter = rightOpticalCounter = 0;
      if (leftMotorHighValue < 255)
      {
        leftMotorHighValue += 5;
      }
      knownCommand = true;
      break;
    case CONTROL_5:
      Serial.print("leftMotorHighValue: "); Serial.print(leftMotorHighValue);
      Serial.print(" rightMotorHighValue: "); Serial.println(rightMotorHighValue);
      leftOpticalCounter = rightOpticalCounter = 0;
      if (rightMotorHighValue < 255)
      {
        rightMotorHighValue += 5;
      }
      knownCommand = true;
      break;
    case CONTROL_7:
      Serial.print("leftMotorHighValue: "); Serial.print(leftMotorHighValue);
      Serial.print(" rightMotorHighValue: "); Serial.println(rightMotorHighValue);
      leftOpticalCounter = rightOpticalCounter = 0;
      if (leftMotorHighValue > 0)
      {
        leftMotorHighValue -= 5;
      }
      knownCommand = true;
      break;
    case CONTROL_8:
      Serial.print("leftMotorHighValue: "); Serial.print(leftMotorHighValue);
      Serial.print(" rightMotorHighValue: "); Serial.println(rightMotorHighValue);
      leftOpticalCounter = rightOpticalCounter = 0;
      if (rightMotorHighValue > 0)
      {
        rightMotorHighValue -= 5;
      }
      knownCommand = true;
      break;
  }
  
  if (!knownCommand && irCommand > 0 && lastIrCommand != 0)
  {
    //Serial.println("!knownCommand && irCommand > 0 && lastIrCommand != 0");
    lastIrCommand = 0;
  }
  if (knownCommand && lastIrCommand == 0)
  {
    //Serial.println("knownCommand && lastIrCommand == 0");
    knownCommand = false;
  }
  
  unsigned long now = millis();
  if (knownCommand)
  {
    //Serial.println("Update timestamp");
    timestamp = now;
  }
  if (now - timestamp <= OPERATION_TIME_MS)
  {
    //Serial.print("now: "); Serial.println(now);
    //Serial.print("timestamp: "); Serial.println(timestamp);
    analogWrite(LEFT_MOTOR, leftMotorState);
    analogWrite(RIGHT_MOTOR, rightMotorState);
  }
  else
  { 
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0);
  }

  if (now - clock1000 >= 1000)
  {
    Serial.print("opticalCounter left "); Serial.print(leftOpticalCounter);
    Serial.print(" right "); Serial.println(rightOpticalCounter);
    clock1000 = now;
  }
  if (now - clock100 >= 100)
  {
    CheckOpticalValue(LEFT_OPTICAL_PIN, lastLeftOpticalValue, leftOpticalCounter);
    CheckOpticalValue(RIGHT_OPTICAL_PIN, lastRightOpticalValue, rightOpticalCounter);
    clock100 = now;
  }
}

void CheckOpticalValue(byte pin, boolean& lastValue, int& counter)
{
  boolean const value = digitalRead(pin);
  if (value != lastValue)
  {
    ++counter;
  }
  lastValue = value;
}