#include <Arduino.h>
#include <HX711.h>
int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;




// HX711_top circuit wiring
const int LOADCELL_DOUT_PIN_top = 7;
const int LOADCELL_SCK_PIN_top = 6;


// HX711_bottom circuit wiring
const int LOADCELL_DOUT_PIN_bot = 5;
const int LOADCELL_SCK_PIN_bot = 4;

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
  scale_top.begin(LOADCELL_DOUT_PIN_top, LOADCELL_SCK_PIN_top);
  scale_bot.begin(LOADCELL_DOUT_PIN_bot, LOADCELL_SCK_PIN_bot);

  Serial.println("Before setting up the scale:");
  Serial.print("read scale_top: \t\t");
  Serial.println(scale_top.read());			// print a raw reading from the ADC

  Serial.print("read scale_bot: \t\t");
  Serial.println(scale_bot.read());			// print a raw reading from the ADC


  Serial.print("read average: \t\t");
  Serial.println(scale_top.read_average(20));  	// print the average of 20 readings from the ADC
  Serial.println(scale_bot.read_average(20));

  Serial.print("get value: \t\t");
  Serial.println(scale_top.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight (not set yet)
  Serial.println(scale_bot.get_value(5));

  Serial.print("get units: \t\t");
  Serial.println(scale_top.get_units(5), 1);
  Serial.println(scale_bot.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare weight (not set) divided
						// by the SCALE parameter (not set yet)

  scale_top.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale_bot.set_scale(2280.f);
  scale_top.tare();				        // reset the scale to 0
  scale_bot.tare();

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale_top.read());                 // print a raw reading from the ADC
  Serial.println(scale_bot.read());
  Serial.print("read average: \t\t");
  Serial.println(scale_top.read_average(20));       // print the average of 20 readings from the ADC
  Serial.println(scale_bot.read_average(20));

  Serial.print("get value: \t\t");
  Serial.println(scale_top.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight, set with tare()
  Serial.println(scale_bot.get_value(5));

  Serial.print("get units: \t\t");
  Serial.println(scale_top.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
	Serial.println(scale_bot.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
						// by the SCALE parameter set with set_scale
					// by the SCALE parameter set with set_scale

  Serial.println("Readings:");

  // Pump set up
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);
}

void loop() {
  Serial.print("one reading:\t");
  Serial.print(scale_top.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale_top.get_units(10), 1);

  scale_top.power_down();			        // put the ADC in sleep mode
  delay(5000);
  scale_top.power_up();

  Serial.print("one reading:\t");
  Serial.print(scale_bot.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale_bot.get_units(10), 1);

  scale_bot.power_down();			        // put the ADC in sleep mode
  delay(5000);
  scale_bot.power_up();
  // put your main code here, to run repeatedly:   

  //Controlling speed (0 = off and 255 = max speed):
  analogWrite(9, 100); //ENA pin
  analogWrite(10, 200); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
}