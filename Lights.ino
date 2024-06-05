#include <Adafruit_NeoPixel.h>
#include "Lights.h"




float batt_voltage = 0;
float v6_voltage = 0;
int analog_val = 0;



// setup rgb leds
Adafruit_NeoPixel batt_leds(batt_led_num, batt_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel IR_status_leds(IR_status_led_num, IR_status_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel status_leds(status_leds_num, status_leds_pin, NEO_GRB + NEO_KHZ800);


void Get_Voltages()
{
  analog_val = analogRead(batt_mon_pin);               // read voltage on pin
  batt_voltage = analog_val * (5.00 / 1023.00) * 2;    // calculate volts/bits and *2 for volt. divider
  batt_voltage = batt_voltage + (0.05 * batt_voltage); // calibration factor

  analog_val = analogRead(v5_mon_pin);
  v6_voltage = (analog_val * (5.00 / 1023.00) * 2);
  v6_voltage = v6_voltage + (0.05* v6_voltage);

  /* print results
  Serial.print("Battery Voltage = ");
  Serial.print(batt_voltage);
  Serial.println(" V");
  Serial.print("6V Voltage = ");
  Serial.print(v6_voltage);
  Serial.println(" V");
  */

  // show battery leds
  if (batt_voltage > 7)
  {
    for (int i = 0; i < batt_led_num; i++)
      batt_leds.setPixelColor(i, 0, 10, 0);
  }
  if (batt_voltage > 6.8 && batt_voltage <= 7)
  {
    batt_leds.setPixelColor(batt_led_num - 1, 5, 5, 0);
    for (int i = 0; i < batt_led_num - 1; i++)
      batt_leds.setPixelColor(i, 0, 10, 0);
  }
  if (batt_voltage > 6.6 && batt_voltage <= 6.8)
  {
    batt_leds.setPixelColor(batt_led_num - 2, 0, 10, 0);
    for (int i = 1; i < batt_led_num; i++)
      batt_leds.setPixelColor(i, 5, 5, 0);
  }
  if (batt_voltage > 6.4 && batt_voltage <= 6.6)
  {
    for (int i = 0; i < batt_led_num; i++)
      batt_leds.setPixelColor(i, 5, 5, 0);
  }
  if (batt_voltage > 6.2 && batt_voltage <= 6.4)
  {
    batt_leds.setPixelColor(batt_led_num - 1, 10, 0, 0);
    for (int i = 0; i < batt_led_num - 1; i++)
      batt_leds.setPixelColor(i, 5, 5, 0);
  }
  if (batt_voltage <= 6.2)
  {
    for (int i = 0; i < batt_led_num; i++)
      batt_leds.setPixelColor(i, 10, 0, 0);
  }
  batt_leds.show();

  // Show 6v fault light
  if (v6_voltage < 5 || v6_voltage > 7.5)
  {
    digitalWrite(v5_fault_pin, HIGH);
  }
  else
    digitalWrite(v5_fault_pin, LOW);
}




