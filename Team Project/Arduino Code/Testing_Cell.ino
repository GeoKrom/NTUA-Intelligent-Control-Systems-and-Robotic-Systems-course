// Calibrating the load cell
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 6;


const float scaling_factor = 829.427083333; // TODO: Fill that again

HX711 scale;

void setup() {

  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_offset(216673); 
  delay(1000);
  scale.set_scale(scaling_factor); 

}

void loop() {

  if (scale.is_ready()) {
    scale.set_offset(216673); 
    delay(1000);
    scale.set_scale(scaling_factor);  
    Serial.print("Offset: ");
    Serial.println(scale.get_offset());
    Serial.print("Place a known weight on the scale...");
    delay(2000);
    long reading = scale.get_units(10);
    Serial.print("Sensor Reading: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}