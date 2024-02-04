#include <BasicLinearAlgebra.h>
#include <HX711.h>
#include <stdio.h>

// Pin definitions for load cells
#define SCK_PIN_1 6
// SCK pin for the first HX711 module
#define DT_PIN_1 7
// DT pin for the first HX711 module
#define SCK_PIN_2 8
// SCK pin for the second HX711 module
#define DT_PIN_2 9
// DT pin for the second HX711 module
// Pin definitions for water pump
const int ENA = 10; // PWM
HX711 scale1;
HX711 scale2;

// Sensor Offset
#define SCALE1_FACTOR 829.427083333
#define SCALE1_OFFSET 216673

#define PI 3.1415926535897932384626433832795
#define V_LOW 0
#define V_HIGH 5
#define V_BINARY_LOW 0
#define V_BINARY_HIGH 255

// Define System Parameters
#define a1 PI *pow(2 * pow(10, -3), 2)
#define a2 PI *pow(2 * pow(10, -3), 2)
#define R 0.04
#define A PI *pow(R, 2)
#define g 9.81
#define Km 240 * pow(10, -3) / (5 * 3600)
#define rho 1

// Define MIT Rule Controller Parameters
#define gamma1 500
#define gamma2 180
#define gamma3 150
#define dt 0.01 // in [sec]

const float theta1 = ((a1 / A) * sqrt(2 * g));
const float theta2 = ((a2 / A) * sqrt(2 * g));
const float b = Km / A;
const float h_max = pow(10, -3) / A;
const float h1_e = h_max / 2;
const float h2_e = pow(theta2 / theta1, 2) * h1_e;
const float u_e = (theta1 / b) * sqrt(h1_e);
const float a2m = 0.05;
const float a1m = 2 * sqrt(a2m);
const int system_state_len = 13;
float system_state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Global Variable
bool enabled = false;

void setup() {
  Serial.begin(9600);
  // Setup load cells
  scale1.begin(DT_PIN_1, SCK_PIN_1);
  scale2.begin(DT_PIN_2, SCK_PIN_2);
  // Setup motor driver
  pinMode(ENA, OUTPUT);

  // Setup upper load cell
  scale1.set_scale(SCALE1_FACTOR);
  delay(1000);
  scale1.set_offset(SCALE1_OFFSET);

  // Setup lower load cell

  // Initial state
  system_state[0] = h1_e;
  system_state[1] = h2_e;
  system_state[4] = 200;
  system_state[5] = 60;
  system_state[6] = 150;

  //
  scale1.set_scale(1);
  scale2.set_scale(2);
}

void loop() {

  initialization();
  // Control Loop
  if (enabled) {
    readAndPrintWeights();
    float pump_value = mitRule();
    // Transform to [V]
    int volt_pump_value =
        map(pump_value, V_LOW, V_HIGH, V_BINARY_LOW, V_BINARY_HIGH);
    sendPump(volt_pump_value);
    delay(dt * 1000);
  }
}

void readAndPrintWeights() {
  // Read the weights from load cells
  float weight1 = scale1.get_units(10);
  float weight2 = scale2.get_units(10);

  Serial.print("Load cell sensor 1");
  Serial.print(": ");
  Serial.print(weight1, 2);
  Serial.println(" kg");

  Serial.print("Load cell sensor 2");
  Serial.print(": ");
  Serial.print(weight2, 2);
  Serial.println(" kg");

  float h1 = weight1 / (A * g * rho);
  float h2 = weight2 / (A * g * rho);

  system_state[0] = h1;
  system_state[1] = h2;

  return;
}

void initialization() {

  float h1 = 0;
  float h2 = 0;
  sendPump(255);
  while (h1 < h1_e || h2 < h2_e) {
    Serial.print("Get scale 1: ");
    Serial.println(scale1.get_scale());

    Serial.print("Get scale 2: ");
    Serial.println(scale2.get_scale());

    // Read the weights from load cells
    float weight1 = scale1.get_units(10);
    float weight2 = scale2.get_units(10);

    Serial.print("Load cell sensor 1");
    Serial.print(": ");
    Serial.print(weight1 - SCALE1_OFFSET, 2);
    Serial.println(" kg");

    Serial.print("Load cell sensor 2");
    Serial.print(": ");
    Serial.print(weight2, 2);
    Serial.println(" kg");

    float h1 = weight1 / (A * g * rho);
    float h2 = weight2 / (A * g * rho);
  }
  enabled = false;
}

void sendPump(int value) { analogWrite(ENA, value); }

float mitRule() {

  float r = ref() - h2_e;
  BLA::Matrix<2, 2> Am = {0, 1, -a2m, -a1m};
  BLA::Matrix<2, 1> Bm = {0, b};

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

  BLA::Matrix<2, 1> x_m = {xm, xm_dot};
  BLA::Matrix<2, 1> dxm = Am * x_m + Bm * r;
  BLA::Matrix<2, 1> filter_1 = {filter1, filter1_dot};
  BLA::Matrix<2, 1> dfilter1 = Am * filter_1 + Bm * r;
  BLA::Matrix<2, 1> filter_2 = {filter2, filter2_dot};
  BLA::Matrix<2, 1> dfilter2 = Am * filter_2 + Bm * x1;
  BLA::Matrix<2, 1> filter_3 = {filter3, filter3_dot};
  BLA::Matrix<2, 1> dfilter3 = Am * filter_3 + Bm * x2;

  // Control input
  float u = theta1_est * r - theta2_est * x1 - theta3_est * x2;

  double e = x2 - x_m(0);

  float dtheta1 = -gamma1 * e * filter1;
  float dtheta2 = gamma2 * e * filter2;
  float dtheta3 = gamma3 * e * filter3;

  // Compute new state
  system_state[0] = h1;
  system_state[1] = h2;
  system_state[2] = xm + dt * dxm(0);
  system_state[3] = xm_dot + dt * dxm(1);
  system_state[4] = theta1_est + dtheta1 * dt;
  system_state[5] = theta2_est + dtheta2 * dt;
  system_state[6] = theta3_est + dtheta3 * dt;
  system_state[7] = filter1 + dt * dfilter1(0);
  system_state[8] = filter1_dot + dt * dfilter1(1);
  system_state[9] = filter2 + dt * dfilter2(0);
  system_state[10] = filter2_dot + dt * dfilter2(1);
  system_state[11] = filter3 + dt * dfilter3(0);
  system_state[12] = filter3_dot + dt * dfilter3(1);

  return;
}

double ref() { return 1.0; }