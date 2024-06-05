#include <Adafruit_NeoPixel.h>

#define v5_mon_pin 23   // Uno A5 -> Spark 23/PF0
#define batt_mon_pin 22 // Uno A4 -> Spark 22/PF1
#define v5_fault_pin 2  // Uno 7 -> Spark 2/Pd1
#define batt_led_pin 0  // Uno 12 -> Spark 0/PD2
#define batt_led_num 3  // number of pins in array
#define IR_status_led_pin 1  // Uno 12 -> Spark 1/PD3
#define IR_status_led_num 8  // number of pins in array
#define status_leds_pin 10 // Spark 10/PB6
#define status_leds_num 3 // number of pins in array


// setup rgb leds
extern Adafruit_NeoPixel batt_leds;
extern Adafruit_NeoPixel IR_status_leds;
extern Adafruit_NeoPixel status_leds;


void Get_Voltages();