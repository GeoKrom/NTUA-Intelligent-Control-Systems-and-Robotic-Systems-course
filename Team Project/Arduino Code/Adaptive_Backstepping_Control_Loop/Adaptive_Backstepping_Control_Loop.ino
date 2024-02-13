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
#define a1 PI *pow(2 * pow(10, -3), 2)
#define a2 PI *pow(2 * pow(10, -3), 2)
// #define R 0.04 
#define R 0.0775/2 // in [m]
#define A PI *pow(R, 2) // in [m^2]
#define g 9.81 // [m/s^2]
#define Km 240 * pow(10, -3) / (5 * 3600)
#define rho 1*pow(10,3) // [kg/m^2]

// Define Adaptive Backstepping Controller Parameters
#define gamma1 0.000025
#define gamma2 0.000035
#define gamma3 0.55
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
const int system_state_len = 5;
float system_state[] = {0, 0, 0, 0, 0};

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
  system_state[2] = 0.01;
  system_state[3] = 0.01;
  system_state[4] = 0.002;

}

void loop() {

 if (!enabled) {
    initialization();
  }
  // Control Loop
  if (enabled) {
    readTopLoadCellMeasurement();
    readBotLoadCellMeasurement();
    float pump_value = adaptiveBackstepping();
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

float adaptiveBackstepping() {

  float r = ref() - h2_e;
  BLA::Matrix<2, 2> Am = {0, 1, -K1, -K2};
  BLA::Matrix<2, 1> Bm = {0, b};
  BLA::Matrix<2, 2> P = {5.6461, 10, 10, 23.4787};
  // Decompose state variables
  float h1 = system_state[0];
  float h2 = system_state[1];
  float theta1_est = system_state[2];
  float theta2_est = system_state[3];
  float b_est = system_state[4];

  // Apply Adaptive Backstepping Controller
  float ksi1 = h2 - r;
  float ksi2 = K1*ksi1 + theta1_est*sqrt(h1) - theta2_est*sqrt(h2);
  
  float alpha2 = K1*(-K1*ksi1 + ksi2) + 0.5*(pow(theta2_est, 2) - pow(theta1_est,2)) - 0.5*theta1_est*theta2_est*sqrt(h1/h2);
  float beta1 = K1*sqrt(h1) - 0.5*theta1_est - 0.5*theta2_est*sqrt(h1/h2);
  float beta2 = -K1*sqrt(h2) + 0.5*theta2_est;
 
  float dtheta1 = gamma1*(ksi1*sqrt(h1) + ksi2*beta1);
  float dtheta2 = gamma2*(-ksi1*sqrt(h2) + ksi2*beta2);


  if(theta1_est <= 0.00001 && dtheta1 < 0){
    dtheta1 = 0;
  }

  if(theta2_est <= 0.00001 && dtheta2 < 0){
    dtheta2 = 0;
  }


  float alpha = -ksi1 - alpha2 - dtheta1*sqrt(h1) + dtheta2*sqrt(h2) - K2*ksi2;
  float beta = 2*sqrt(h1)/(b_est*theta1_est);
  float u = alpha*beta;

  float beta3 = theta1_est*(1/(2*sqrt(h2)))*u;
  float dbeta = gamma3*ksi2*beta3;

  if (b_est <= 0.00001 && dbeta < 0){
    dbeta = 0;
  }


  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = theta1_est + dtheta1 * dt;
  system_state[3] = theta2_est + dtheta2 * dt;
  system_state[4] = b_est + dbeta * dt;

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