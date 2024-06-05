#include "Motor.h"
#include <pins_arduino.h>

volatile int16_t LW_Current = 0;
volatile int16_t RW_Current = 0;

// Function to initialise the Motors
void Motor_Init()
{
  // Set Spped and Direction pins as outputs, speed set to 0%

  // Motor A 
  pinMode(LW_Dir, OUTPUT);
  pinMode(LW_Speed, OUTPUT);
  digitalWrite(LW_Dir, HIGH); // Set Motor A Direction HIGH = FWD
  analogWrite(LW_Speed, 108);
  // Motor B
  pinMode(RW_Dir, OUTPUT);
  pinMode(RW_Speed, OUTPUT);
  digitalWrite(RW_Dir, HIGH); // Set Motor B Direction LOW = FWD
  analogWrite(RW_Speed, 100);
}

// Function to handle a passed speed correction
void Speed_Adjust(int16_t Left, int16_t Right)
{
  int16_t RW_Adjust;
  int16_t LW_Adjust;

  // Check if each wheel needs to be adjusted up or down in speed

  if (Left < LW_Current)
  {
    LW_Adjust = LW_Current - Left;
  }
  else if (Left > LW_Current)
  {
    LW_Adjust = LW_Current + Left;
  }
  else if (Left == 0)
  {
    LW_Adjust = LW_Current;
  }

  if (Right < RW_Current)
  {
    RW_Adjust = RW_Current - Right;
  }
  else if (Right > RW_Current)
  {
    RW_Adjust = RW_Current + Right;
  }
  else if (Right == 0)
  {
    RW_Adjust = RW_Current;
  }

  // call the speed_set function to implement the changes in speed or direction if needed
  Speed_Set(LW_Adjust, RW_Adjust);
}

// Function to handle setting the speed of the motors
void Speed_Set(int16_t Left, int16_t Right)
{
  Serial.print("Speed set requested for Left = ");
  Serial.print(Left);
  Serial.print(" & Right = ");
  ;
  Serial.println(Right);
  // A negative motor speed means backwards, positive is forwards.

  // Chceck if motor speed is negative and set directions if it needs to be changed
  if (Left < 0)
  {
    digitalWrite(LW_Dir, LOW); // Set Motor A Direction
  }
  else
    digitalWrite(LW_Dir, HIGH);

  if (Right < 0)
  {
    digitalWrite(RW_Dir, HIGH); // Set Motor B Direction
  }
  else
    digitalWrite(RW_Dir, LOW);

  // Set speeds
  analogWrite(LW_Speed, abs(Left));
  analogWrite(RW_Speed, abs(Right));

  // Store Speeds
  LW_Current = Left;
  RW_Current = Right;
}
