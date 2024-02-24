#include <BasicLinearAlgebra.h>
#include <HX711.h>
#include <stdio.h>

#define PI 3.1415926535897932384626433832795
#define V_LOW 0
#define V_HIGH 5
#define V_BINARY_LOW 0
#define V_BINARY_HIGH 255

// HX711_top circuit wiring
const int LOADCELL_DOUT_PIN_top = 7;
const int LOADCELL_SCK_PIN_top = 6;
const float scale_top_value = 854.48241206; // divide with [gr]

HX711 scale_top;

// HX711_bottom circuit wiring
const int LOADCELL_DOUT_PIN_bot = 9;
const int LOADCELL_SCK_PIN_bot = 8;
const float scale_bot_value = 142479.4/199; // divide with [gr]

HX711 scale_bot;

// Pin definitions for water pump
const int ENA = 10; // PWM
const int MOTOR1_PIN1 = 2; // 
const int MOTOR1_PIN2 = 3;

// Define System Parameters
#define a1 PI *pow(2 * pow(10, -3), 2) // why is pow(2 * pow)
#define a2 PI *pow(2 * pow(10, -3), 2)
// #define R 0.04 
#define R 0.0775/2 // in [m]
#define A PI *pow(R, 2) // in [m^2]
#define g 9.81 // [m/s^2]
#define Km 240 * pow(10, -3) / (5 * 3600)
#define rho 1*pow(10,3) // [kg/m^2]

// Define MIT Rule Controller Parameters
#define gamma1 10000
#define gamma2 10000
#define gamma3 10000
#define dt 0.1 // in [sec]

const float theta1 = ((a1 / A) * sqrt(2 * g));
const float theta2 = ((a2 / A) * sqrt(2 * g));
const float b = Km / A;
// const float h_max = pow(10, -3) / A;
const float h_max = 0.04;
const float h1_e = h_max / 4;
const float h2_e = pow(theta2 / theta1, 2) * h1_e;
const float u_e = (theta1 / b) * sqrt(h1_e);
const float a2m = 1;
const float a1m = 2 * sqrt(a2m);
const int system_state_len = 13;
float system_state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Global Variable
bool enabled;
char userInput;

void setup() {
  Serial.begin(9600);

  // Setup load cell (TOP)
  scale_top.begin(LOADCELL_DOUT_PIN_top, LOADCELL_SCK_PIN_top);
  scale_top.set_scale(scale_top_value);
  scale_top.tare();

  // Setup load cell (BOTTOM)
  scale_bot.begin(LOADCELL_DOUT_PIN_bot, LOADCELL_SCK_PIN_bot);
  scale_bot.set_scale(scale_bot_value); 
  scale_bot.tare();

  // Setup motor driver
  pinMode(ENA, OUTPUT);
  pinMode(MOTOR1_PIN1,OUTPUT);
  pinMode(MOTOR1_PIN2,OUTPUT);
  // Provide Direction
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  
  // Initial state
  // system_state[0] = h1_e;
  // system_state[1] = h2_e;
  system_state[4] = 200;
  system_state[5] = 60;
  system_state[6] = 150;

}

void loop() {
  if(Serial.available()> 0){ 
    // Serial.print('1');
    userInput = Serial.read();     
    if(userInput == 'g'){  
      // Serial.print('2');
      if (!enabled) {
        initialization();
      }
      // Control Loop
      if (enabled) {
        readTopLoadCellMeasurement();
        readBotLoadCellMeasurement();
        float pump_value = mitRule();
        // Transform to [V]
        int volt_pump_value =
            map(pump_value, V_LOW, V_HIGH, V_BINARY_LOW, V_BINARY_HIGH);
        if (volt_pump_value<0){volt_pump_value = 0;}
        sendPump(volt_pump_value);
        delay(dt * 1000);
        Serial.print(system_state[1]*1000);
        Serial.print(",");
        Serial.print((system_state[2]+h2_e)*1000);
        Serial.print(",");
        Serial.print(millis());
        Serial.print(",");
        Serial.print("volt");
        Serial.print(",");
        Serial.print(volt_pump_value);
        Serial.print(",");
      }
    }
    else if (userInput == 'c'){
      sendPump(0);
    }

  }
  // else{
  //   Serial.print('3');
  //   sendPump(0);
  // }
}

void initialization() {

  float h1 = 0;
  float h2 = 0;
  sendPump(90); // MAX 255 
  while (h1 < h1_e || h2 < h2_e) {
  // while (true){
    h1 = readTopLoadCellMeasurement();
    h2 = readBotLoadCellMeasurement();
  }
  system_state[0] = h1;
  system_state[1] = h2;
  enabled = true;
  Serial.println("Target Reached");

  return;
}

// *==== Control Functions ====*

float mitRule() {

  float r = ref() - h2_e;

  // Decompose state variables
  float h1 = system_state[0];
  float h2 = system_state[1];
  float xm = system_state[2];
  float xm_dot = system_state[3];
  float theta1_est = system_state[4];
  float theta2_est = system_state[5];
  float theta3_est = system_state[6];
  float filter1 = system_state[7];
  float filter1_dot = system_state[8];
  float filter2 = system_state[9];
  float filter2_dot = system_state[10];
  float filter3 = system_state[11];
  float filter3_dot = system_state[12];

  
  // Apply MIT RUle Controller
  float x1 = h1 - h1_e;
  float x2 = h2 - h2_e;
  // float x1 = h1;
  // float x2 = h2;


  float dxm0 = xm_dot;
  float dxm1 = -a2m*xm - a1m*xm_dot + a2m*r;
  float dfilter1_0 = filter1_dot;
  float dfilter1_1 = -a2m*filter1 - a1m*filter1_dot + a2m*r;
  float dfilter2_0 = filter2_dot;
  float dfilter2_1 = -a2m*filter2 - a1m*filter2_dot + a2m*x1;
  float dfilter3_0 = filter3_dot;
  float dfilter3_1 = -a2m*filter3 - a1m*filter3_dot + a2m*x2;

  
  // Control input
  float u = theta1_est * r - theta2_est * x1 - theta3_est * x2;

  float e = x2 - xm;

  float dtheta1 = -gamma1 * e * filter1;
  float dtheta2 = gamma2 * e * filter2;
  float dtheta3 = gamma3 * e * filter3;
  

  

  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = xm + dt * dxm0;
  system_state[3] = xm_dot + dt * dxm1;
  system_state[4] = theta1_est + dtheta1 * dt;
  system_state[5] = theta2_est + dtheta2 * dt;
  system_state[6] = theta3_est + dtheta3 * dt;
  system_state[7] = filter1 + dt * dfilter1_0;
  system_state[8] = filter1_dot + dt * dfilter1_1;
  system_state[9] = filter2 + dt * dfilter2_0;
  system_state[10] = filter2_dot + dt * dfilter2_1;
  system_state[11] = filter3 + dt * dfilter3_0;
  system_state[12] = filter3_dot + dt * dfilter3_1;

  Serial.print("law");
  Serial.print(",");  
  Serial.print(u);
  Serial.print(","); 
  Serial.print("error");
  Serial.print(",");
  Serial.print(e*1000);
  Serial.print(",");
  Serial.print("state: ");
  Serial.print(",");
  Serial.print(h1*1000);
  Serial.print(",");
  Serial.print(h2*1000);
  Serial.print(",");
  Serial.print(xm*1000);
  Serial.print(",");
  Serial.print(xm_dot*1000);
  Serial.print(",");
  Serial.print(theta1_est*1000);
  Serial.print(",");
  Serial.print(theta2_est*1000);
  Serial.print(",");
  Serial.print(theta3_est*1000);
  Serial.print(",");
  Serial.print(filter1*1000);
  Serial.print(",");
  Serial.print(filter1_dot*1000);
  Serial.print(",");
  Serial.print(filter2*1000);
  Serial.print(",");
  Serial.print(filter2_dot*1000);
  Serial.print(",");
  Serial.print(filter3*1000);
  Serial.print(",");
  Serial.println(filter3_dot*1000);
  

  return u;
}

double ref() { return h_max/2; }

// *==== SENSOR FUNCTIONS ====*

float readTopLoadCellMeasurement() {
  // Read from top load cell
  float top_load_cell_measurement = scale_top.get_units(1)/1000; // in [Kg]
  float h1 = top_load_cell_measurement / (A * rho);  // in [m]
  system_state[0] = h1;

  return h1;
}

float readBotLoadCellMeasurement() {
  // Read from bot load cell
  float bot_load_cell_measurement = scale_bot.get_units(1)/1000; // in [Kg]  
  float h2 = bot_load_cell_measurement / (A * rho); // in [m]
  system_state[1] = h2;

  // TODO: Print height 

  return h2;
}

void sendPump(int value) { analogWrite(ENA, value); }
