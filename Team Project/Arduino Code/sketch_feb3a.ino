// Calibrating the load cell
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 6;

// 
// const int LOADCELL_DOUT_PIN = 9;
// const int LOADCELL_SCK_PIN = 8;

HX711 scale;

void setup() {
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
     
  Serial.println("Tare... remove any weights from the scale.");
  delay(1000);
  scale.tare(100);
  scale.set_scale(1);  
}

void loop() {

  if (scale.is_ready()) {
    Serial.print("Offset: ");
    Serial.println(scale.get_offset());
    delay(1000);
    long reading = scale.get_units(10);
    Serial.print("Un-Scaled Result: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(5000);
}

//calibration factor will be the (reading)/(known weight)
