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
float h2_cur;
float h2_prev;

HX711 scale_bot;

// Pin definitions for water pump
const int ENA = 10; // PWM
const int MOTOR1_PIN1 = 2; // 
const int MOTOR1_PIN2 = 3;

// time constanst
float time_prev = 0.0;
float time_cur = 0.0;


// Define System Parameters
#define a1 PI *pow(2 * pow(10, -3), 2)
#define a2 PI *pow(2 * pow(10, -3), 2)
// #define R 0.04 
#define R 0.0775/2 // in [m]
#define A PI *pow(R, 2) // in [m^2]
#define g 9.81 // [m/s^2]
#define Km 240 * pow(10, -3) / (5 * 3600)
#define rho 1*pow(10,3) // [kg/m^2]

// Define Adaptive Feedback Linearization Controller Parameters
#define gammax 500000
#define gammar 100000
#define dt 0.01 // in [sec]

const float theta1 = ((a1 / A) * sqrt(2 * g));
const float theta2 = ((a2 / A) * sqrt(2 * g));
const float b = Km / A;
//const float h_max = pow(10, -3) / A;
const float h_max = 0.06;
const float h1_e = h_max / 4;
const float h2_e = pow(theta2 / theta1, 2) * h1_e;
const float u_e = (theta1 / b) * sqrt(h1_e);
const float a2m = 4;
const float a1m = 2 * sqrt(a2m);
const float K1 = a2m;
const float K2 = 2*sqrt(a2m);
const int system_state_len = 7;
float system_state[] = {0, 0, 0, 0, 0, 0, 0};

// Global Variable
bool enabled = false;
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
  system_state[4] = -40;
  system_state[5] = -12;
  system_state[6] = 24;

}

void loop() {
    if(Serial.available()> 0){ 
        // Serial.print('1');
        userInput = Serial.read();     
        if(userInput == 'g'){  
    
  if (!enabled) {
      initialization();
    }
    // Control Loop
    if (enabled) {
      h2_prev = system_state[1];
      readTopLoadCellMeasurement();
      readBotLoadCellMeasurement();
      h2_cur = system_state[1];
      time_cur = millis();
      float h2_dot = (h2_cur - h2_prev)/(time_cur - time_prev);
      float pump_value = MRAC(h2_dot);
      if (pump_value<0){pump_value = 0;}
      if (pump_value>5){pump_value = 3;}
      // Transform to [V]
      int volt_pump_value =
          map(pump_value, V_LOW, V_HIGH, V_BINARY_LOW, V_BINARY_HIGH);
      
      sendPump(volt_pump_value);
      delay(dt * 1000);
      h2_prev = system_state[1];
      time_prev = millis();
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

float MRAC(float h2_dot) {

  float r = ref() - h2_e;
  // BLA::Matrix<2, 2> Am = {0, 1, -a2m, -a1m};
  // BLA::Matrix<2, 1> Bm = {0, b};
  // BLA::Matrix<2, 2> P = {5.6461, 10, 10, 23.4787};
  float P11 = 5.6461;
  float P12 = 10;
  float P22 = 23.4787;

  // Decompose state variables
  // Decompose state variables
  float h1 = system_state[0];
  float h2 = system_state[1];
  float xm = system_state[2];
  float xm_dot = system_state[3];
  float Kx1_est = system_state[4];
  float Kx2_est = system_state[5];
  float Kr_est = system_state[6];
  
  float dxm0 = xm_dot;
  float dxm1 = -a2m*xm - a1m*xm_dot + a2m*r ;
  
  // Apply MRAC Controller
  float e0 = h2 - h2_e - xm;
  float e1 = h2_dot - xm_dot;
  float dKx0 = -gammax*(h2-h2_e)*(e0*P12 + e1*P22);
  float dKx1 = -gammax*h2_dot*(e0*P12 + e1*P22);
  float dKr = -gammar*r*(e0*P12 + e1*P22);
  
  // Projection
  if(Kr_est <= 0 && dKr < 0){
    dKr = 0;
  }
  if(Kr_est >= 1000 && dKr > 0){
    dKr = 0;
  }
  if(Kx1_est >= 0 && dKx0 > 0){
    dKx0 = 0;
  }
  if(Kx1_est <= -1000 && dKx0 < 0){
    dKx0 = 0;
  }

  if(Kx2_est >= 0 && dKx1 > 0){
    dKx1 = 0;
  }
  if(Kx2_est <= -1000 && dKx1 < 0){
    dKx1 = 0;
  }

  
  float u = Kx1_est*(h2-h2_e) + Kx2_est*h2_dot + Kr_est*r + 1.76;

  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = xm + dt * dxm0;
  system_state[3] = xm_dot + dt * dxm1;
  system_state[4] = Kx1_est + dKx0 * dt;
  system_state[5] = Kx2_est + dKx1 * dt;
  system_state[6] = Kr_est + dKr * dt;

Serial.print("law");
  Serial.print(",");  
  Serial.print(u);
  Serial.print(","); 
  Serial.print("error");
  Serial.print(",");
  Serial.print(e0*1000);
  Serial.print(",");
  Serial.print(e1*1000);
  Serial.print(",");
  Serial.print("state: ");
  Serial.print(",");
  // Serial.print(system_state[0]*1000);
  // Serial.print(",");
  // Serial.print(system_state[1]*1000);
  // Serial.print(",");
  // Serial.print(system_state[2]*1000);
  // Serial.print(",");
  // Serial.print(system_state[3]*1000);
  // Serial.print(",");
  Serial.print(system_state[4]*1000);
  Serial.print(",");
  Serial.print(system_state[5]*1000);
  Serial.print(",");
  Serial.print(system_state[6]*1000);
  Serial.print(",");
  Serial.print(system_state[6]*1000);
  Serial.print(",");
  Serial.print(dKx0*1000);
  Serial.print(",");
  Serial.print(dKx1*1000);
  Serial.print(",");
  Serial.println(dKr*1000);
  
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