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

// Define Adaptive Feedback Linearization Controller Parameters
#define gamma1 0.00035
#define gamma2 0.00025
#define gamma3 0.00055
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
float h1_prev = 0.0;
float h2_prev = 0.0;
float h1_cur = 0.0;
float h2_cur = 0.0;
float delta_const = 0.005;
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
  system_state[0] = h1_e;
  system_state[1] = h2_e;
  system_state[2] = h2_e;
  system_state[3] = 0;
  system_state[4] = 0.0105;
  system_state[5] = 0.0105;
  system_state[6] = 0.002;

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
    h1_prev = system_state[0];
    h2_prev = system_state[1];
    readTopLoadCellMeasurement();
    readBotLoadCellMeasurement();
    h1_cur = system_state[0];
    h2_cur = system_state[1];
    if(abs(h1_cur - h1_prev)>delta_const){system_state[0] = h1_prev;}
    if(abs(h2_cur - h2_prev)>delta_const){system_state[1] = h2_prev;}
    int pump_value = adaptiveFeedbackLinearization();
    // Transform to [V]
    if (pump_value<V_LOW){pump_value=V_LOW;}
    else if (pump_value>V_HIGH){pump_value=V_HIGH-1;}
    
    float volt_pump_value =
        floatMap(pump_value, V_LOW, V_HIGH, V_BINARY_LOW, V_BINARY_HIGH);



    // if (volt_pump_value<V_BINARY_LOW){volt_pump_value = V_BINARY_LOW;}
    // else if (volt_pump_value>V_BINARY_HIGH){volt_pump_value = V_BINARY_HIGH-20;}
    
    sendPump(floor(volt_pump_value));
    delay(dt * 1000);
    Serial.print(system_state[1]*1000);
    Serial.print(",");
    Serial.print(system_state[2]*1000);
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

float adaptiveFeedbackLinearization() {


  float r = ref();
  BLA::Matrix<2, 2> Am = {0, 1, -K1, -K2};
  BLA::Matrix<2, 1> Bm = {0, K1};
  BLA::Matrix<2, 2> P = {5.6461, 10, 10, 23.4787};
  // Decompose state variables
  float h1 = system_state[0];
  float h2 = system_state[1];
  float xm = system_state[2];
  float xm_dot = system_state[3];
  float theta1_est = system_state[4];
  float theta2_est = system_state[5];
  float b_est = system_state[6];
  
  // Apply Adaptive Feedback Linearization Controller
  float z1 = h2;
  float z2 = theta1_est*sqrt(h1) - theta2_est*sqrt(h2);

  BLA::Matrix<2,1> x_m = {xm, xm_dot};
  BLA::Matrix<2,1> dxm = Am * x_m + Bm * r;
  BLA::Matrix<2,1> e = {z1 - xm, z2 - xm_dot};
  float v = 0 - K1*e(0) - K2*e(1);
  BLA::Matrix<1,2> trH1 = {sqrt(h1), -0.5*theta1_est - 0.5*theta2_est*sqrt(h1/h2)};
  BLA::Matrix<1,2> trH2 = {-sqrt(h2), 0.5*theta2_est};

  BLA::Matrix<1,1> H1 = trH1*P*e;
  BLA::Matrix<1,1> H2 = trH2*P*e;
  float dtheta1 = gamma1*H1(0);
  float dtheta2 = gamma2*H2(0);


  if(theta1_est <= 0.001 && dtheta1 < 0){
    dtheta1 = 0;
  }

  if(theta2_est <= 0.001 && dtheta2 < 0){
    dtheta2 = 0;
  }


  float alpha = -dtheta1*sqrt(h1) + dtheta2*sqrt(h2) + 0.5*theta2_est*(theta1_est*sqrt(h1/h2) - theta2_est);

  float beta = theta1_est*sqrt(h1)/b_est;

  float u = (alpha + v)*2*sqrt(h1)/(b_est*theta1_est) + beta;

  BLA::Matrix<1, 2> trH3 = {0, theta1_est*(1/(2*sqrt(h1)))*u};
  BLA::Matrix<1, 1> H3 = trH3*P*e;
  float dbeta = gamma3*H3(0);

  if (b_est <= 0.001 && dbeta < 0){
    dbeta = 0;
  }


  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = xm + dt * dxm(0);
  system_state[3] = xm_dot + dt * dxm(1);
  system_state[4] = theta1_est + dtheta1 * dt;
  system_state[5] = theta2_est + dtheta2 * dt;
  system_state[6] = b_est + dbeta * dt;
  
  Serial.print("law");
  Serial.print(",");  
  Serial.print(u);
  Serial.print(","); 
  Serial.print("error");
  Serial.print(",");
  Serial.print(e(0)*1000);
  Serial.print(",");
  Serial.print(e(1)*1000);
  Serial.print(",");
  Serial.print("state param: ");
  Serial.print(",");
  Serial.print(system_state[0]*1000);
  Serial.print(",");
  Serial.print(system_state[1]*1000);
  Serial.print(",");
  // Serial.print(system_state[2]*1000);
  // Serial.print(",");
  // Serial.print(system_state[3]*1000);
  // Serial.print(",");
  Serial.print(system_state[4]*1000000);
  Serial.print(",");
  Serial.print(system_state[5]*1000000);
  Serial.print(",");
  Serial.print(system_state[6]*1000000);
  Serial.print(",");
  Serial.print(dtheta1*1000000);
  Serial.print(",");
  Serial.print(dtheta2*1000000);
  Serial.print(",");
  Serial.println(dbeta*1000000);


  return floor(u);
}

double ref() { return h_max/3; }

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

float floatMap(float x, float inMin, float inMax, float outMin, float outMax){
  return (x-inMin)*(outMax-outMin)/(inMax-inMin)+outMin;
}