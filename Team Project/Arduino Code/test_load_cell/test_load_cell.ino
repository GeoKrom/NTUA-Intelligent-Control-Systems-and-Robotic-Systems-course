#include <Arduino.h>
#include <HX711.h>

// HX711_top circuit wiring
const int LOADCELL_DOUT_PIN_top = 7;
const int LOADCELL_SCK_PIN_top = 6;

// HX711_bottom circuit wiring
const int LOADCELL_DOUT_PIN_bot = 9;
const int LOADCELL_SCK_PIN_bot = 8;

const float scale_top_value = 854.48241206; // divide with [gr]
const float scale_bot_value = 142479.4/199; // divide with [gr]

HX711 scale_top;
HX711 scale_bot;

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  // scale_top.reset();

  // *=== TOP LOAD CELL ===*
  scale_top.begin(LOADCELL_DOUT_PIN_top, LOADCELL_SCK_PIN_top);
  
  scale_top.set_scale(scale_top_value); // this value is obtained by calibrating the scale with known weights; see the README for details
  scale_top.tare(); // reset the scale to 0

  // *=== BOT LOAD CELL ===*

  scale_bot.begin(LOADCELL_DOUT_PIN_bot, LOADCELL_SCK_PIN_bot);
  scale_bot.set_scale(scale_bot_value); // this value is obtained by calibrating the scale with known weights; see the README for details
  scale_bot.tare(); // reset the scale to 0

  Serial.println("Readings:");
}

void loop() {
  Serial.print("scale_top reading:\t");
  Serial.print(scale_top.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale_top.get_units(10), 1);

  Serial.println("----------------------------------");

  Serial.print("scale_bot reading:\t");
  Serial.print(scale_bot.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale_bot.get_units(10), 1);

  Serial.println("================================");

  delay(1000);
}