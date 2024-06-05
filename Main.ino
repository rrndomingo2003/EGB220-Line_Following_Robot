#include "Motor.h"
#include "Lights.h"
#include <Adafruit_NeoPixel.h>

int current_error = 0;
int last_error = 0;
int basespeed_left = 89;
int basespeed_right = 85;
const int default_basespeed_left = 89;
const int default_basespeed_right = 85;
int motorspeed_left = 0;
int motorspeed_right = 0;
int current_position;

float Kp = 0.08;  
float Ki = 0.0002;
float Kd = 0.4;   
int P;
int I;
int D;
int lastError = 0;

int red_line_times = 1; 
bool S8;
bool S7;
bool S6;
bool S5;
bool S4;
bool S3;
bool S2;
bool S1;

bool SW2;
bool SW1;

bool bw_tog;

bool obstacle;
int IR;
int coloursensor;
int stop_start;

int switch_var;
unsigned long current_time;
unsigned long colour_timer;
unsigned long white_line_timer;
unsigned long colour_debounce;
unsigned long stop_timer;
char current_colour;
char prev_colour;
char debounced_colour;
char prev_side_IR;
char last_debounced_colour = 'b';
char current_SideIRSensor = 'b';
char debounced_side_IR;
char last_debounced_side_IR = 'b';
unsigned long IR_timer;
int SideIRSensor;
int Side_IR_debounce_counter = 0;

unsigned long voltages_timer = 0;
unsigned long rgb_timer = 0;
uint16_t rgb_counter = 0;

void setup()
{
  rgb_timer = millis();
  voltages_timer = millis();
  colour_timer = millis();
  white_line_timer = millis();
  pinMode(v5_mon_pin, INPUT);
  pinMode(batt_mon_pin, INPUT);
  pinMode(v5_fault_pin, OUTPUT);
  Motor_Init();

  // initialise leds
  batt_leds.begin();
  status_leds.begin();
  for (int i = 0; i < batt_led_num; i++)
    batt_leds.setPixelColor(i, 0, 0, 0);
  batt_leds.show();
  IR_status_leds.begin();
  for (int i = 0; i < IR_status_led_num; i++)
    IR_status_leds.setPixelColor(i, 0, 0, 0);
  IR_status_leds.show();

  Serial.begin(9600);

  // Set the sensor pin to input
  pinMode(14, INPUT); // S8 PD4 D4
  pinMode(12, INPUT); // S7 PD6 D12
  pinMode(6, INPUT);  // S6 PD7 D6
  pinMode(8, INPUT);  // S5 PB4 D8
  pinMode(18, INPUT); // S4 PF7 D18
  pinMode(19, INPUT); // S3 PF6 D19
  pinMode(20, INPUT); // S2 PF5 D20
  pinMode(15, INPUT); // S1 PF4 D21

  // Set switch pins
  pinMode(5, INPUT);  // SW2 PC6 D5
  pinMode(13, INPUT); // SW1 PC7 D13

  // Set Side-sensor pins
  pinMode(A6, INPUT);        // A5		D23 	PF0
  pinMode(A3, INPUT_PULLUP); // A3		D21 	PF4	 IR

  // Default case
  switch_var = 0;
  for (int i = 0; i < status_leds_num; i++)
  {
    status_leds.setPixelColor(i, 0, 30, 0);
  }
  status_leds.show();
}

void PID()
{
  current_error = 3500 - current_position;
  D = current_error - last_error;
  last_error = current_error;
  int motorspeed = current_error * Kp + D * Kd; // assume motorspeed = 1500*0.07=105
  int motorspeed_left = basespeed_left + motorspeed;   // motor left speed needed=88+105=193
  int motorspeed_right = basespeed_right - motorspeed; // motor right speed needed=80-105=-25
  const uint8_t maxspeed = 255;
  if (motorspeed_left > maxspeed)
  {
    motorspeed_left = maxspeed;
  }
  if (motorspeed_right > maxspeed)
  {
    motorspeed_right = maxspeed;
  }
  if (motorspeed_left < 0)
  {
    motorspeed_left = 0;
  }
  if (motorspeed_right < 0)
  {
    motorspeed_right = 0;
  }

  // Serial.print("motorspeed_left");
  // Serial.println(motorspeed_left);
  // Serial.print("motorspeed_right");
  // Serial.println(motorspeed_right);

  // Motor A
  digitalWrite(LW_Dir, HIGH);
  analogWrite(LW_Speed, motorspeed_left);
  // Motor B
  digitalWrite(RW_Dir, HIGH);
  analogWrite(RW_Speed, motorspeed_right);
}

void loop()
{
  Get_Voltages();

  S8 = digitalRead(14);
  S7 = digitalRead(12);
  S6 = digitalRead(6);
  S5 = digitalRead(8);
  S4 = digitalRead(18);
  S3 = digitalRead(19);
  S2 = digitalRead(20);
  S1 = digitalRead(15);

  SW2 = digitalRead(5);
  SW1 = digitalRead(13);

  obstacle = digitalRead(9);

  SideIRSensor = analogRead(A3);
  // Serial.print("IR: ");
  // Serial.println(IR);
  if (millis() - IR_timer >= 5)
  {
    Serial.print("Reading Side IR Sensor = ");
    Serial.println(SideIRSensor);
    if (SideIRSensor <= 550) // WHITE
    {
      Serial.println("Side IR Sensor white detected");
      current_SideIRSensor = 'w';
    }
    if (SideIRSensor >= 550) // BLACK
    {
      current_SideIRSensor = 'b';
      Serial.println("Side IR Sensor black detected");
    }

    // if the Side IR hasn't changed then start a counter
    if (prev_side_IR == current_SideIRSensor)
    {
      Side_IR_debounce_counter++;
    }
    else if (prev_side_IR != current_SideIRSensor)
    {
      Side_IR_debounce_counter = 0;
    }
    Serial.print(" IR sample =");
    Serial.println(Side_IR_debounce_counter);
    prev_side_IR = current_SideIRSensor;
    IR_timer = millis();

    // if 20 readings of the same colour in a row then update the debounced colour
    if (Side_IR_debounce_counter == 20)
    {
      debounced_side_IR = current_SideIRSensor;
      Serial.print("debounced side ir = ");
      Serial.println(debounced_side_IR);
      Side_IR_debounce_counter = 0;
    }

    if (debounced_side_IR == 'b')
    {
      Serial.println("set IR LED side to black");
      status_leds.setPixelColor(1, 0, 0, 120);
      status_leds.show();
    }
    if (debounced_side_IR == 'w')
    {
      Serial.println("set IR LED side to white");
      status_leds.setPixelColor(1, 40, 40, 40);
      status_leds.show();
    }
  }

  coloursensor = analogRead(A6);

  // read the colour every 100ms
  if (millis() - colour_timer >= 5) // 1
  {
    Serial.print("Reading Colour Sensor = ");
    Serial.println(coloursensor);
    if (coloursensor <= 100) // WHITE
    {
      Serial.println("white detected");
      current_colour = 'w';
    }
    if (coloursensor >= 825) // BLACK
    {
      current_colour = 'b';
      Serial.println("black detected");
    }
    if (coloursensor >= 700 && coloursensor < 825) // Red
    {
      Serial.println("Red detected");
      current_colour = 'r';
    }
    if (coloursensor > 100 && coloursensor <= 200) // Green
    {
      current_colour = 'g';
      Serial.println("green detected");
    }

    // if the colour hasn't changed then start a counter
    if (prev_colour == current_colour)
    {
      colour_debounce++;
    }
    else if (prev_colour != current_colour)
    {
      colour_debounce = 0;
    }
    Serial.print("sample =");
    Serial.println(colour_debounce);
    prev_colour = current_colour;
    colour_timer = millis();

    // if 11 readings of the same colour in a row then update the debounced colour
    if (colour_debounce == 11) // 50
    {
      debounced_colour = current_colour;
      Serial.print("debounced colour = ");
      Serial.println(debounced_colour);
      colour_debounce = 0;
    }

    // if debounced black line occurs
    if (debounced_colour == 'b')
    {
      bw_tog = 0;
      status_leds.setPixelColor(2, 0, 0, 120);
      status_leds.show();
    }

    // if debounced red line occurs
    if (debounced_colour == 'r')
    {
      status_leds.setPixelColor(2, 120, 0, 0);
      status_leds.show();
    }

    // if debounced green line occurs
    if (debounced_colour == 'g')
    {
      status_leds.setPixelColor(2, 0, 120, 0);
      status_leds.show();
      basespeed_left = default_basespeed_left;
      basespeed_right = default_basespeed_right;
      switch_var = 0;
    }

    ///////////slow zone red line detection//////////

    if (debounced_colour == 'r' && current_colour == 'r' && S1 == 1 && S2 == 1 && S8 == 1 && S7 == 1) // only do if going straight and very red
    {
      // go slow
      switch_var = 3;
    }
    ///////////////////////// Start Stop Detection/////////////////////

    // If a debounced white line occurs after a black space
    if (debounced_colour == 'w' && current_colour == 'w' && bw_tog == 0 && S1 == 1 && S2 == 1 && S8 == 1 && S7 == 1) // WHITE
    {
      status_leds.setPixelColor(2, 40, 40, 40);
      status_leds.show();
      bw_tog = 1;
      Serial.print("Colour Sensor White Line detected with colour sens val = ");
      Serial.println(coloursensor);
      stop_start += 1;
    }
    if (stop_start == 4) // 4
    {
      Serial.println("Colour Sensor White Line detected twice so stopping");

      switch_var = 2;
      stop_timer = millis();
      Serial.print("switch var =");
      Serial.println(switch_var);
      Serial.print("stop timer =");
      Serial.println(stop_timer);
      stop_start = 0;
    }
  }

  ////////////////////curve///////////////////////////
  if (IR < 100 && current_position < 3200 || current_position > 3800)
  {
    basespeed_left = 20;
    basespeed_right = 20;
  }

  else
  {
    basespeed_left = default_basespeed_left;
    basespeed_right = default_basespeed_right;
  }


  switch (switch_var)
  {

  case 0:
    // Serial.println("Case 0");
    if (S1 == 0)
    {
      current_position = 0;
    }
    if (S8 == 0)
    {
      current_position = 7000;
    }
    if (S2 == 0 && S1 == 0)
    {
      current_position = 500;
    }

    if (S7 == 0 && S8 == 0)
    {
      current_position = 6500;
    }
    if (S3 == 0 && S2 == 0)
    {
      current_position = 1500;
    }
    if (S7 == 0 && S6 == 0)
    {
      current_position = 5500;
    }
    if (S4 == 0 && S3 == 0)
    {
      current_position = 2500;
    }
    if (S5 == 0 && S6 == 0)
    {
      current_position = 4500;
    }
    /*If the middle sensors detects white line, continue straight. MOTOR A - LEFT  MOTOR B - RIGHT*/
    if (S5 == 0 && S4 == 0)
    {
      current_position = 3500;
    }

    PID();

    break;

  case 2:
    for (int i = 0; i < status_leds_num; i++)
    {
      status_leds.setPixelColor(0, 120, 0, 0);
    }
    status_leds.show();
    // Serial.println("Case 2");
    while (millis() - stop_timer < 1500)
    {
      S8 = digitalRead(14);
      S7 = digitalRead(12);
      S6 = digitalRead(6);
      S5 = digitalRead(8);
      S4 = digitalRead(18);
      S3 = digitalRead(19);
      S2 = digitalRead(20);
      S1 = digitalRead(15);
      // Serial.println("getting ready to stop");
      Serial.print("Case 2 stop timer =");
      Serial.println(millis() - stop_timer);
      if (S1 == 0)
      {
        current_position = 0;
      }
      if (S8 == 0)
      {
        current_position = 7000;
      }
      if (S2 == 0 && S1 == 0)
      {
        current_position = 500;
      }

      if (S7 == 0 && S8 == 0)
      {
        current_position = 6500;
      }
      if (S3 == 0 && S2 == 0)
      {
        current_position = 1500;
      }
      if (S7 == 0 && S6 == 0)
      {
        current_position = 5500;
      }
      if (S4 == 0 && S3 == 0)
      {
        current_position = 2500;
      }
      if (S5 == 0 && S6 == 0)
      {
        current_position = 4500;
      }
      /*If the middle sensors detects white line, continue straight. MOTOR A - LEFT  MOTOR B - RIGHT*/
      if (S5 == 0 && S4 == 0)
      {
        current_position = 3500;
      }

      PID();
    }

    // if stop timer is at 1 second then stop
    while (millis() - stop_timer >= 1500 && millis() - stop_timer < 3500)
    {
      Serial.println("Stopping");
      // Motor A
      digitalWrite(LW_Dir, HIGH);
      analogWrite(LW_Speed, 0);
      // Motor B
      digitalWrite(RW_Dir, HIGH);
      analogWrite(RW_Speed, 0);
    }

    stop_start = 0;
    Serial.println("Resetting stop/start");
    switch_var = 0;

    break;

  // /////////////////////////Slow Zone//////////////////////////////////
  case 3:
    // Serial.println("Case 0");

    basespeed_left = 55;
    basespeed_right = 55;
    if (S1 == 0)
    {
      current_position = 0;
    }
    if (S8 == 0)
    {
      current_position = 7000;
    }
    if (S2 == 0 && S1 == 0)
    {
      current_position = 500;
    }

    if (S7 == 0 && S8 == 0)
    {
      current_position = 6500;
    }
    if (S3 == 0 && S2 == 0)
    {
      current_position = 1500;
    }
    if (S7 == 0 && S6 == 0)
    {
      current_position = 5500;
    }
    if (S4 == 0 && S3 == 0)
    {
      current_position = 2500;
    }
    if (S5 == 0 && S6 == 0)
    {
      current_position = 4500;
    }
    /*If the middle sensors detects white line, continue straight. MOTOR A - LEFT  MOTOR B - RIGHT*/
    if (S5 == 0 && S4 == 0)
    {
      current_position = 3500;
    }

    PID();

    break;

  default:
    switch_var = 0;
    break;
  }
  // IR LEDs ON
  if (S1 == 0)
  {
    IR_status_leds.setPixelColor(7, 10, 10, 10);
  }
  if (S2 == 0)
  {
    IR_status_leds.setPixelColor(6, 10, 10, 10);
  }
  if (S3 == 0)
  {
    IR_status_leds.setPixelColor(5, 10, 10, 10);
  }
  if (S4 == 0)
  {
    IR_status_leds.setPixelColor(4, 10, 10, 10);
  }
  if (S5 == 0)
  {
    IR_status_leds.setPixelColor(3, 10, 10, 10);
  }
  if (S6 == 0)
  {
    IR_status_leds.setPixelColor(2, 10, 10, 10);
  }
  if (S7 == 0)
  {
    IR_status_leds.setPixelColor(1, 10, 10, 10);
  }
  if (S8 == 0)
  {
    IR_status_leds.setPixelColor(0, 10, 10, 10);
  }
  // IR LEDs OFF
  if (S1 == 1)
  {
    IR_status_leds.setPixelColor(7, 0, 0, 0);
  }
  if (S2 == 1)
  {
    IR_status_leds.setPixelColor(6, 0, 0, 0);
  }
  if (S3 == 1)
  {
    IR_status_leds.setPixelColor(5, 0, 0, 0);
  }
  if (S4 == 1)
  {
    IR_status_leds.setPixelColor(4, 0, 0, 0);
  }
  if (S5 == 1)
  {
    IR_status_leds.setPixelColor(3, 0, 0, 0);
  }
  if (S6 == 1)
  {
    IR_status_leds.setPixelColor(2, 0, 0, 0);
  }
  if (S7 == 1)
  {
    IR_status_leds.setPixelColor(1, 0, 0, 0);
  }
  if (S8 == 1)
  {
    IR_status_leds.setPixelColor(0, 0, 0, 0);
  }

  IR_status_leds.show();
}