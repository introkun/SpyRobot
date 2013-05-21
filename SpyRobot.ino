#include <IRremote.h>

// Arduino I/O
#define LEFT_OPTICAL_PIN 2
#define RIGHT_OPTICAL_PIN 3
#define RECV_PIN 4
#define LEFT_MOTOR 5
#define RIGHT_MOTOR 6
#define LIGHT_SENSOR_PIN A0

// Interrupts
#define LEFT_OPTICAL_PIN_INTERRUPT 0
#define RIGHT_OPTICAL_PIN_INTERRUPT 1

// IR remote control binding
#define CONTROL_MINUS 0xFFA857
#define CONTROL_PLUS 0xFF906F
#define CONTROL_OFF 0xFFA25D
#define CONTROL_MODE 0xFF629D
#define CONTROL_REPEAT 0xFFFFFFFF
#define CONTROL_4 0xFF10EF
#define CONTROL_5 0xFF38C7
#define CONTROL_7 0xFF42BD
#define CONTROL_8 0xFF4AB5

// Constants
#define OPERATION_TIME_MS 100
#define FEEDBACK_THRESHOLD 5
#define WHEEL_SPEED_STEP 5
#define MAX_ANALOG_VALUE 255

class Robot
{
public:
  struct Operation
  {
    enum Enum
    {
      Stop = 0,
      MoveForward,
      TurnLeft,
      TurnRight
    };
  };

  struct Wheel
  {
    enum Enum
    {
      Left = 0,
      Right
    };
  };

  struct Control
  {
    enum Enum
    {
      SpeedUp = 0,
      SpeedDown
    };
  };

  Robot() :
    m_operation(Operation::Stop),
    m_leftMotorValue(MAX_ANALOG_VALUE),
    m_rightMotorValue(MAX_ANALOG_VALUE),
    m_highLeftMotorValue(MAX_ANALOG_VALUE),
    m_highRightMotorValue(MAX_ANALOG_VALUE)
  {
    //
  }

  void Stop()
  {
    m_operation = Operation::Stop;
    SetupMotors(0, 0);
  }

  void MoveForward()
  {
    m_operation = Operation::MoveForward;
    SetupMotors(m_highLeftMotorValue, m_highRightMotorValue);
  }

  void Left()
  {
    m_operation = Operation::TurnLeft;
    SetupMotors(0, m_highRightMotorValue);
  }

  void Right()
  {
    m_operation = Operation::TurnRight;
    SetupMotors(m_highLeftMotorValue, 0);
  }

  void Operate()
  {
    analogWrite(LEFT_MOTOR, m_leftMotorValue);
    analogWrite(RIGHT_MOTOR, m_rightMotorValue);
  }

  void StopOperate()
  {
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0);
  }

  void SetupWheel(Wheel::Enum wheel, Control::Enum control, int const value = WHEEL_SPEED_STEP)
  {
    Serial.print("1. SetupWheel left "); Serial.print(m_highLeftMotorValue);
    Serial.print(" right  "); Serial.println(m_highRightMotorValue);
    if (control == Control::SpeedUp)
    {
      switch (wheel)
      {
        case Wheel::Left:
          m_highLeftMotorValue += value;
          break;
        case Wheel::Right:
          m_highRightMotorValue += value;
          break;
      }
    }
    else
    {
      switch (wheel)
      {
        case Wheel::Left:
          m_highLeftMotorValue -= value;
          break;
        case Wheel::Right:
          m_highRightMotorValue -= value;
          break;
      }
    }

    m_highLeftMotorValue = constrain(m_highLeftMotorValue, 0, MAX_ANALOG_VALUE);
    m_highRightMotorValue = constrain(m_highRightMotorValue, 0, MAX_ANALOG_VALUE);

    Serial.print("2. SetupWheel left "); Serial.print(m_highLeftMotorValue);
    Serial.print(" right  "); Serial.println(m_highRightMotorValue);
  }

  void CheckFeedback(unsigned long const& now, int const leftOpticalCounter,
    int const rightOpticalCounter)
  {
    if (leftOpticalCounter !=0 && rightOpticalCounter != 0)
    {
      Serial.print("opticalCounter left "); Serial.print(leftOpticalCounter);
      Serial.print(" right "); Serial.println(rightOpticalCounter);
      Serial.print("pwm left "); Serial.print(m_highLeftMotorValue);
      Serial.print(" right "); Serial.println(m_highRightMotorValue);
    }

    if (m_operation == Operation::MoveForward &&
      abs(leftOpticalCounter - rightOpticalCounter) >= FEEDBACK_THRESHOLD)
    {
      if (leftOpticalCounter > rightOpticalCounter)
      {
        Serial.println("left wheel is to fast");
        if (m_highRightMotorValue > m_highLeftMotorValue &&
          m_highRightMotorValue < MAX_ANALOG_VALUE)
        {
          Serial.println("SpeedUp Right");
          SetupWheel(Wheel::Right, Control::SpeedUp);
        }
        else
        {
          Serial.println("SpeedDown Left");
          SetupWheel(Wheel::Left, Control::SpeedDown);
        }
      }
      else
      {
        Serial.println("Right wheel is to fast");
        if (m_highLeftMotorValue > m_highRightMotorValue &&
          m_highLeftMotorValue < MAX_ANALOG_VALUE)
        {
          Serial.println("SpeedUp Left");
          SetupWheel(Wheel::Left, Control::SpeedUp);
        }
        else
        {
          Serial.println("SpeedDown Right");
          SetupWheel(Wheel::Right, Control::SpeedDown);
        }
      }
    }
  }

private:
  void SetupMotors(byte const leftValue, byte const rightValue)
  {
    m_leftMotorValue = leftValue;
    m_rightMotorValue = rightValue;
  }

private:
  Operation::Enum m_operation;
  int m_leftMotorValue;
  int m_rightMotorValue;
  int m_highLeftMotorValue;
  int m_highRightMotorValue;
};

Robot robot;
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long lastIrCommand = 0;
unsigned long timestamp = 0;

unsigned long opticalCheckTimer;
volatile int leftOpticalCounter = 0;
volatile int rightOpticalCounter = 0;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(LEFT_OPTICAL_PIN, INPUT);
  pinMode(RIGHT_OPTICAL_PIN, INPUT);
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  opticalCheckTimer = millis();
  attachInterrupt(LEFT_OPTICAL_PIN_INTERRUPT, CounterLeft, RISING);
  attachInterrupt(RIGHT_OPTICAL_PIN_INTERRUPT, CounterRight, RISING);
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
      robot.Right();
      knownCommand = true;
      break;
    case CONTROL_MINUS:
      robot.Left();
      knownCommand = true;
      break;
    case CONTROL_OFF:
      robot.Stop();
      knownCommand = true;
      break;
    case CONTROL_MODE:
      robot.MoveForward();
      knownCommand = true;
      break;
    case CONTROL_REPEAT:
      knownCommand = true;
      break;
    case CONTROL_4:
      robot.SetupWheel(Robot::Wheel::Left, Robot::Control::SpeedUp);
      knownCommand = true;
      break;
    case CONTROL_5:
      robot.SetupWheel(Robot::Wheel::Right, Robot::Control::SpeedUp);
      knownCommand = true;
      break;
    case CONTROL_7:
      robot.SetupWheel(Robot::Wheel::Left, Robot::Control::SpeedDown);
      knownCommand = true;
      break;
    case CONTROL_8:
      robot.SetupWheel(Robot::Wheel::Right, Robot::Control::SpeedDown);
      knownCommand = true;
      break;
  }
  
  if (!knownCommand && irCommand > 0 && lastIrCommand != 0)
  {
    lastIrCommand = 0;
  }
  if (knownCommand && lastIrCommand == 0)
  {
    knownCommand = false;
  }
  
  int lightSendor= analogRead(LIGHT_SENSOR_PIN);
  Serial.println(lightSendor);
  if (lightSendor < 700)
  {
    robot.Right();
    knownCommand = true;
  }

  unsigned long now = millis();
  if (knownCommand)
  {
    timestamp = now;
  }
  if (now - timestamp <= OPERATION_TIME_MS)
  {
    robot.Operate();
  }
  else
  { 
    robot.StopOperate();
  }

  if (now - opticalCheckTimer >= 1000)
  {
    robot.CheckFeedback(now, leftOpticalCounter, rightOpticalCounter);
    leftOpticalCounter = 0;
    rightOpticalCounter = 0;
    opticalCheckTimer = now;
  }
}

void CounterLeft()
{
  ++leftOpticalCounter;
}

void CounterRight()
{
  ++rightOpticalCounter;
}