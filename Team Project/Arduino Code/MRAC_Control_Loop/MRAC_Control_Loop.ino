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
#define gammax 500
#define gammar 100
#define dt 0.01 // in [sec]

const float theta1 = ((a1 / A) * sqrt(2 * g));
const float theta2 = ((a2 / A) * sqrt(2 * g));
const float b = Km / A;
//const float h_max = pow(10, -3) / A;
const float h_max = 0.06;
const float h1_e = h_max / 2;
const float h2_e = pow(theta2 / theta1, 2) * h1_e;
const float u_e = (theta1 / b) * sqrt(h1_e);
const float a2m = 0.05;
const float a1m = 2 * sqrt(a2m);
const float K1 = a2m;
const float K2 = 2*sqrt(a2m);
const int system_state_len = 7;
float system_state[] = {0, 0, 0, 0, 0, 0, 0};

// Global Variable
bool enabled = false;

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
  system_state[0] = h1_e;
  system_state[1] = h2_e;
  system_state[4] = -20;
  system_state[5] = -200;
  system_state[6] = 15;

}

void loop() {

 if (!enabled) {
    initialization();
  }
  // Control Loop
  if (enabled) {
    h2_prev = system_state[1];
    readTopLoadCellMeasurement();
    readBotLoadCellMeasurement();
    h2_cur = system_state[1];
    float h2_dot = (h2_cur - h2_prev)/dt;
    float pump_value = MRAC(h2_dot);
    // Transform to [V]
    int volt_pump_value =
        map(pump_value, V_LOW, V_HIGH, V_BINARY_LOW, V_BINARY_HIGH);
    sendPump(volt_pump_value);
    delay(dt * 1000);
  }
}

void initialization() {

  float h1 = 0;
  float h2 = 0;
  sendPump(90); // MAX 255 
  // while (h1 < h1_e || h2 < h2_e) {
  while (true){
    h1 = readTopLoadCellMeasurement();
    h2 = readBotLoadCellMeasurement();
  }
  enabled = true;
  Serial.println("Target Reached");

  return;
}

// *==== Control Functions ====*

float MRAC(float h2_dot) {

  float r = ref() - h2_e;
  BLA::Matrix<2, 2> Am = {0, 1, -a2m, -a1m};
  BLA::Matrix<2, 1> Bm = {0, b};
  BLA::Matrix<2, 2> P = {5.6461, 10, 10, 23.4787};
  
  // Decompose state variables
  float h1 = system_state[0];
  float h2 = system_state[1];
  float xm = system_state[2];
  float xm_dot = system_state[3];
  float Kx1_est = system_state[4];
  float Kx2_est = system_state[5];
  float Kr_est = system_state[6];

  BLA::Matrix<1, 2> Kx_est = {Kx1_est, Kx2_est};
  // Apply MRAC Controller
  BLA::Matrix<2, 1> x_m = {xm, xm_dot};
  BLA::Matrix<2, 1> dxm = Am * x_m + Bm * r;
  BLA::Matrix<2, 1> B = {0, 1};
  BLA::Matrix<2, 1> x = {h2 - h2_e, h2_dot};
  BLA::Matrix<2, 1> e = x - x_m;
  BLA::Matrix<1, 2> e_transpose = {e(0),e(1)};
  BLA::Matrix<2, 2> Gamma_x = {gammax,0,0,gammax};
  BLA::Matrix<1, 1> gamma_r = {gammar};
  BLA::Matrix<2, 1> dKx = -Gamma_x*x*e_transpose*P*B;
  BLA::Matrix<1, 1> dKr = -gamma_r*r*e_transpose*P*B;
  
  // Projection
  if(Kr_est <= 0 && dKr(0) < 0){
    dKr(0) = 0;
  }
  if(Kr_est >= 1000 && dKr(0) > 0){
    dKr(0) = 0;
  }

  if(Kx1_est >= 0 && dKx(0) > 0){
    dKx(0) = 0;
  }
  if(Kx1_est <= -1000 && dKx(0) < 0){
    dKx(0) = 0;
  }

  if(Kx2_est >= 0 && dKx(0) > 0){
    dKx(0) = 0;
  }
  if(Kx2_est <= -1000 && dKx(0) < 0){
    dKx(0) = 0;
  }

  
  BLA::Matrix<1, 1> u_c = Kx_est*x + Kr_est*r;
  float u = u_c(0);
  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = xm + dt * dxm(0);
  system_state[3] = xm_dot + dt * dxm(1);
  system_state[4] = Kx1_est + dKx(0) * dt;
  system_state[5] = Kx2_est + dKx(1) * dt;
  system_state[6] = Kr_est + dKr(0) * dt;

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