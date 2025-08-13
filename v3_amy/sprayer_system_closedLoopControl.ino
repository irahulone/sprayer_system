// user input is the desired flow rates for each individual nozzle, each number separated by a space
// Ex: 80 100 90 100 

#include <Arduino.h>

#include "flow1_lookup_banked.h"
#include "flow2_lookup_banked.h"
#include "flow3_lookup_banked.h"
#include "flow4_lookup_banked.h"


float lookUpTableFlow1[7] = {0.0, 55.01, 86.55, 116.75, 144.57, 172.91, 189.47};
float lookUpTablePwr1[7] = {0.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0};

float lookUpTableFlow2[7] = {0.0, 52.97, 83.72, 116.94, 145.96, 172.15, 185.17};
float lookUpTablePwr2[7] = {0.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0};

float lookUpTableFlow3[7] = {0.0, 69.89, 97.82, 121.26, 144.74, 170.26, 188.66};
float lookUpTablePwr3[7] = {0.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0};

float lookUpTableFlow4[7] = {0.0, 98.71, 124.63, 149.43, 170.70, 194.78, 208.57};
float lookUpTablePwr4[7] = {0.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0};

float* lookUpTableFlow;
float* lookUpTablePwr;

// true = banked system (2 pumps), false = 1-to-1 system (4 pumps)
bool isBanked = false; 


// Define the valve control pins for M1, M2, M3, M4 (positive terminals) from DF robot 4ch driver shield, DRI0039
// https://wiki.dfrobot.com/Quad_Motor_Driver_Shield_for_Arduino_SKU_DRI0039
// M1-M4 are valve PWM
#define M1 4
#define M2 12
#define M3 8
#define M4 7
// E1-E4 are valve dir 
#define E1 3
#define E2 11
#define E3 5
#define E4 6


// Pump Control Variables
float pump_pwr_uf[4] = { 0.0, 0.0, 0.0, 0.0 };          // unfiltered pump power 
float pump_pwr[4] = { 0.0, 0.0, 0.0, 0.0 };             // Pump Power Value that gets passed into the PUMPS
float valve_pos[4] = { 0.0, 0.0, 0.0, 0.0 };            // Valve position for each valve
float flow_rate[4] = { 0.0, 0.0, 0.0, 0.0 };            // moving average filtered flow rates
float flow_rate_left = 0;                               // left bank filtered flow rate
float flow_rate_right = 0;                              // right bank filtered flow rate
float dfr[4] = {0, 0, 0, 0};                            // desired flow rate for each nozzle
float dfr_L = 0;                                        // combined left bank flow rates
float dfr_R = 0;                                        // combined right bank flow rates 

float p_pump[4] = { 0.0, 0.0, 0.0, 0.0 };               // pump proportional term
float int_pump[4] = { 0.0, 0.0, 0.0, 0.0 };             // pump integral term

int eFlow_pump[4] = { 0, 0, 0, 0 };                     // flow rate error for pump control
double error_i[4] = { 0, 0, 0, 0 };                     // integral error for pump control
float kp_pump[4] = {0.0, 0.0, 0.0, 0.0};                // create pump proportional gain variable
float ki_pump[4] = {0.0, 0.0, 0.0, 0.0};                // create pump integral gain variable

float calc_u_base = 0.0;                                // placeholder variable for calculated open loop pump power
float u_base[4] = { 0.0, 0.0, 0.0, 0.0 };               // Pump Pwr calculated via Lookup Table
float u_PI[4] = { 0.0, 0.0, 0.0, 0.0 };                 // Pump Pwr calculated via PI Controller 

// Nozzle Control Variables
int eFlow[4] = { 0, 0, 0, 0 };                            // flow rate error for nozzle control 
double eFlow_i[4] = { 0, 0, 0, 0 };                       // integral error for nozzle control
float kp_nozzle[4] = { 0.0, 0.0, 0.0, 0.0 };              // create nozzle proportional gain variable
float ki_nozzle[4] = { 0.0, 0.0, 0.0, 0.0 };              // create nozzle integral gain variable

float nozzle_PI[4] = { 0.0, 0.0, 0.0, 0.0 };


int zone_id = 0;
long pulses[4] = { 0, 0, 0, 0 };

const int window_size = 10;  // define the window size og: 400
int flow_s0_ar[window_size];
int flow_s1_ar[window_size];
int flow_s2_ar[window_size];
int flow_s3_ar[window_size];

const int window_size_pump = 10; // og: 70
int pump_1_ar[window_size_pump];

unsigned long cloopTime; // to measure flow rate
unsigned long cloopTime2; // to compute moving average
unsigned long currentT;
unsigned long cloopT; // for sampling time, main loop


// Function to set the valve position based on the input valve number and voltage (0-10V)
void set_valve(int valve, int voltage) {
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  digitalWrite(M3, LOW);
  digitalWrite(M4, HIGH);

  if (voltage < 0) voltage = 0;
  if (voltage > 100) voltage = 100;

  // Map the voltage range (0-10V) to the PWM range (0-255)
  int pwmValue = map(voltage, 0, 100, 0, 255);

  // Set the motor driver to output the PWM signal for the specified valve
  if (valve == 1) {
    analogWrite(E1, pwmValue);
    //Serial.print("Valve 1 set to voltage: ");
  } else if (valve == 2) {
    analogWrite(E2, pwmValue);
    //Serial.print("Valve 2 set to voltage: ");
  } else if (valve == 3) {
    analogWrite(E3, pwmValue);
    //Serial.print("Valve 3 set to voltage: ");
  } else if (valve == 4) {
    analogWrite(E4, pwmValue);
    //Serial.print("Valve 4 set to voltage: ");
  }
}

/*
void ctrl_pump(int ch = 0, int val = 0) {
  int min_val_pos = 20;
  if (ch == 1) {
    if (valve_pos[0] < min_val_pos && valve_pos[1] < min_val_pos)
      val = 0;
  }
  if (ch == 2) {
    if (valve_pos[2] < min_val_pos && valve_pos[3] < min_val_pos)
      val = 0;
  }

  int max_per = 60;
  if (val < 0) val = 0;
  else if (val > max_per) val = max_per;
  int v = map(val, 0, 100, 0, 1000);

  // pump #2 is banked for nozzles 1&2
  if (ch == 1) {
    Serial2.print("!G 2 ");
    Serial2.print(String(v));
    Serial2.print("_\r");
  }

  // pump #4 is banked for nozzles 3&4
  if (ch == 2) {
    Serial3.print("!G 2 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }
}
*/
void ctrl_pump(int ch = 0, int val = 0) {
  int min_val_pos = 20;
  int max_per = 60;
  if (val < 0) val = 0;
  else if (val > max_per) val = max_per;
  int v = map(val, 0, 100, 0, 1000);

  if (ch == 1) {
    if (valve_pos[0] < min_val_pos)
      val = 0;
  }
  if (ch == 2) {
    if (valve_pos[1] < min_val_pos)
      val = 0;
  }
  if (ch == 3) {
    if (valve_pos[2] < min_val_pos)
      val = 0;
  }
  if (ch == 4) {
    if (valve_pos[3] < min_val_pos)
      val = 0;
  }

  if (ch == 1) {
    Serial2.print("!G 1 ");
    Serial2.print(String(v));
    Serial2.print("_\r");
  }

  if (ch == 2) {
    Serial2.print("!G 2 ");
    Serial2.print(String(v));
    Serial2.print("_\r");
  }

  if (ch == 3) {
    Serial3.print("!G 1 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }

  if (ch == 4) {
    Serial3.print("!G 2 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }
}


void debug_disp()
{
//  Serial.print("des"); Serial.print("\t");
//  Serial.print(des_flow_rate[0], 0); Serial.print("\t"); Serial.print(des_flow_rate[1], 0); Serial.print("\t");
//  //Serial.print(des_flow_rate[2], 0); Serial.print("\t"); Serial.print(des_flow_rate[3], 0); Serial.print("\t");
//  Serial.println();
  Serial.println();
  Serial.print("des"); Serial.print("\t");
  Serial.print(dfr[0]); Serial.print("\t");
  Serial.print(dfr[1]); Serial.print("\t");
  Serial.print(dfr[2]); Serial.print("\t");
  Serial.print(dfr[3]); Serial.print("\t");
  Serial.println();
  /*
  Serial.print("flo L/R bank"); Serial.print("\t");
  Serial.print(flow_rate_left); Serial.print("\t");
  Serial.print(flow_rate_right); Serial.print("\t");
  Serial.println();
  */
  Serial.print("vlv"); Serial.print("\t");
  Serial.print(valve_pos[0],0); Serial.print("\t"); 
  Serial.print(valve_pos[1],0); Serial.print("\t");
  Serial.print(valve_pos[2],0); Serial.print("\t"); 
  Serial.print(valve_pos[3],0); Serial.print("\t");
  Serial.println();

  Serial.print("flo"); Serial.print("\t");
  Serial.print(flow_rate[0], 0); Serial.print("\t"); 
  Serial.print(flow_rate[1], 0); Serial.print("\t");
  Serial.print(flow_rate[2], 0); Serial.print("\t"); 
  Serial.print(flow_rate[3], 0); Serial.print("\t");
  Serial.println();

  Serial.print("pmp"); Serial.print("\t");
  Serial.print(pump_pwr[0],2); Serial.print("\t"); 
  Serial.print(pump_pwr[1], 2); Serial.print("\t");
  Serial.print(pump_pwr[2],2); Serial.print("\t"); 
  Serial.print(pump_pwr[3], 2); Serial.print("\t");
  Serial.println();
  
  //Serial.print("Zone: "); Serial.println(zone_id);

  Serial.print("int error"); Serial.print("\t"); 
  Serial.print(error_i[0],2); Serial.print("\t");
  Serial.print(error_i[1],2); Serial.print("\t");
  Serial.print(error_i[2],2); Serial.print("\t");
  Serial.print(error_i[3],2); Serial.print("\t");
  Serial.println();

  Serial.print("int"); Serial.print("\t");
  Serial.print(int_pump[0], 2); Serial.print("\t");
  Serial.print(int_pump[1], 2); Serial.print("\t");
  Serial.print(int_pump[2], 2); Serial.print("\t");
  Serial.print(int_pump[3], 2); Serial.print("\t");
  Serial.println();

  Serial.print("prop"); Serial.print("\t");
  Serial.print(p_pump[0], 2); Serial.print("\t");
  Serial.print(p_pump[1], 2); Serial.print("\t");
  Serial.print(p_pump[2], 2); Serial.print("\t");
  Serial.print(p_pump[3], 2); Serial.print("\t");
  Serial.println();

  Serial.print("u_PI"); Serial.print("\t");
  Serial.print(u_PI[0], 2); Serial.print("\t");
  Serial.print(u_PI[1], 2); Serial.print("\t");
  Serial.print(u_PI[2], 2); Serial.print("\t");
  Serial.print(u_PI[3], 2); Serial.print("\t");
  Serial.println();
}

void send2ros()
{
  for (int i = 0; i < 4; i++) {
    Serial.print(dfr[i], 0); Serial.print(","); 
    Serial.print(flow_rate[i], 0); Serial.print(",");
    Serial.print(pump_pwr[i], 0); Serial.print(",");
    if (i < 3) Serial.print(",");
  }
  Serial.println();
}


void send2Matlab() {
  for (int i = 0; i < 4; i++) {
    Serial.print(pump_pwr[i], 0); Serial.print(",");
    Serial.print(dfr[i], 0); Serial.print(",");
    Serial.print(flow_rate[i], 0); Serial.print(",");
    Serial.print(valve_pos[i], 0);
    if (i < 3) Serial.print(",");
  }
  Serial.println();
}

float maf_pump1(float x)
{
  int p = window_size_pump;
  pump_1_ar[p] = x;
  float a = 0;
  for(int i = 1; i <= p; i++)
    a += pump_1_ar[i];
  a = a/p;
  for(int i = 2; i <= p; i++)
    pump_1_ar[i-1] = pump_1_ar[i];
  return a;
}


void get_flow_rates() {
  if (Serial1.available()) {
    // values read are already filtered on the encoder arduino 
    flow_rate[0] = Serial1.parseFloat();
    flow_rate[1] = Serial1.parseFloat();
    flow_rate[2] = Serial1.parseFloat();
    flow_rate[3] = Serial1.parseFloat();

    //pulses[0] = Serial1.parseInt();
    //pulses[1] = Serial1.parseInt();
    //pulses[2] = Serial1.parseInt();
    //pulses[3] = Serial1.parseInt();
  }
}

// finds min pump power per nozzle and sets u_base=max of those values 
int getPumpPower_L(float valve1, float valve2, float q1_des, float q2_des) {  
  int v1_idx = constrain(int((valve1 - 20) / 5), 0, 8);
  int v2_idx = constrain(int((valve2 - 20) / 5), 0, 8);

  const int num_pwr = 11;
  const float pwr_start = 30.0;
  const float pwr_step = 2.0;

  float f1_vals[num_pwr];
  float f2_vals[num_pwr];

  for (int i = 0; i < num_pwr; i++) {
    f1_vals[i] = pgm_read_float_near(&flow1_lookup_banked[i][v1_idx][v2_idx]);
    f2_vals[i] = pgm_read_float_near(&flow2_lookup_banked[i][v1_idx][v2_idx]);
  }

  // --- Find minimum power for Nozzle 3 ---
  int idx1 = 0;
  while (idx1 < num_pwr - 1 && f1_vals[idx1] < q1_des) {
    idx1++;
  }

  // Interpolate if needed
  float pwr1;
  if (idx1 == 0) {
    pwr1 = pwr_start;
  } else {
    float f_lo = f1_vals[idx1 - 1];
    float f_hi = f1_vals[idx1];
    float t = (q1_des - f_lo) / (f_hi - f_lo + 1e-6);
    pwr1 = pwr_start + (idx1 - 1 + t) * pwr_step;
  }

  // --- Find minimum power for Nozzle 4 ---
  int idx2 = 0;
  while (idx2 < num_pwr - 1 && f2_vals[idx2] < q2_des) {
    idx2++;
  }

  float pwr2;
  if (idx2 == 0) {
    pwr2 = pwr_start;
  } else {
    float f_lo = f2_vals[idx2 - 1];
    float f_hi = f2_vals[idx2];
    float t = (q2_des - f_lo) / (f_hi - f_lo + 1e-6);
    pwr2 = pwr_start + (idx2 - 1 + t) * pwr_step;
  }

  // Return the max of the two
  return int(max(pwr1, pwr2) + 0.5);  // round to nearest int
}

int getPumpPower_R(float valve3, float valve4, float q3_des, float q4_des) {  
  int v3_idx = constrain(int((valve3 - 20) / 5), 0, 8);
  int v4_idx = constrain(int((valve4 - 20) / 5), 0, 8);

  const int num_pwr = 11;
  const float pwr_start = 30.0;
  const float pwr_step = 2.0;

  float f3_vals[num_pwr];
  float f4_vals[num_pwr];

  for (int i = 0; i < num_pwr; i++) {
    f3_vals[i] = pgm_read_float_near(&flow3_lookup_banked[i][v3_idx][v4_idx]);
    f4_vals[i] = pgm_read_float_near(&flow4_lookup_banked[i][v3_idx][v4_idx]);
  }

  // --- Find minimum power for Nozzle 3 ---
  int idx3 = 0;
  while (idx3 < num_pwr - 1 && f3_vals[idx3] < q3_des) {
    idx3++;
  }

  // Interpolate if needed
  float pwr3;
  if (idx3 == 0) {
    pwr3 = pwr_start;
  } else {
    float f_lo = f3_vals[idx3 - 1];
    float f_hi = f3_vals[idx3];
    float t = (q3_des - f_lo) / (f_hi - f_lo + 1e-6);
    pwr3 = pwr_start + (idx3 - 1 + t) * pwr_step;
  }

  // --- Find minimum power for Nozzle 4 ---
  int idx4 = 0;
  while (idx4 < num_pwr - 1 && f4_vals[idx4] < q4_des) {
    idx4++;
  }

  float pwr4;
  if (idx4 == 0) {
    pwr4 = pwr_start;
  } else {
    float f_lo = f4_vals[idx4 - 1];
    float f_hi = f4_vals[idx4];
    float t = (q4_des - f_lo) / (f_hi - f_lo + 1e-6);
    pwr4 = pwr_start + (idx4 - 1 + t) * pwr_step;
  }

  // Return the max of the two
  return int(max(pwr3, pwr4) + 0.5);  // round to nearest int
}
/*
// finds min pump power to satisfy both q3_des and q4_des and interpolates btwn current and prev level to get more accurate power
int getPumpPower_L(float valve1, float valve2, float q1_des, float q2_des) {
  int v1_idx = constrain(int(valve1 / 5) - 4, 0, 8);  // 20–60 mapped to 0–8
  int v2_idx = constrain(int(valve2 / 5) - 4, 0, 8);  // 20–60 mapped to 0–8

  float min_combined_error = 99999;
  int best_idx = 0;

  for (int pwr_idx = 0; pwr_idx < 11; pwr_idx++) {
    float f1 = pgm_read_float_near(&flow1_2d_lookup_banked[pwr_idx][v1_idx]);
    float f2 = pgm_read_float_near(&flow2_2d_lookup_banked[pwr_idx][v2_idx]);

    float err = abs(f1 - q1_des) + abs(f2 - q2_des);
    if (err < min_combined_error) {
      min_combined_error = err;
      best_idx = pwr_idx;
    }
  }

  // Interpolation with next index
  if (best_idx < 10) {
    float f1_lo = pgm_read_float_near(&flow1_2d_lookup_banked[best_idx][v1_idx]);
    float f2_lo = pgm_read_float_near(&flow2_2d_lookup_banked[best_idx][v2_idx]);
    float f1_hi = pgm_read_float_near(&flow1_2d_lookup_banked[best_idx + 1][v1_idx]);
    float f2_hi = pgm_read_float_near(&flow2_2d_lookup_banked[best_idx + 1][v2_idx]);

    float q1_diff = q1_des - f1_lo;
    float q2_diff = q2_des - f2_lo;
    float denom1 = f1_hi - f1_lo;
    float denom2 = f2_hi - f2_lo;

    float w1 = (denom1 != 0) ? q1_diff / denom1 : 0;
    float w2 = (denom2 != 0) ? q2_diff / denom2 : 0;

    float interp_idx = best_idx + (w1 + w2) / 2.0;

    // Clamp index to [0, 10]
    if (interp_idx > 10) interp_idx = 10;
    if (interp_idx < 0) interp_idx = 0;

    // Power index to % mapping: 0 → 30%, 1 → 32%, ..., 10 → 50%
    return int(2 * interp_idx + 30);  // 2% step size
  }

  return 30 + best_idx * 2;
}

int getPumpPower_R(float valve3, float valve4, float q3_des, float q4_des) {
  int v3_idx = constrain(int(valve3 / 5) - 4, 0, 8);  // 20–60 mapped to 0–8
  int v4_idx = constrain(int(valve4 / 5) - 4, 0, 8);  // 20–60 mapped to 0–8

  float min_combined_error = 99999;
  int best_idx = 0;

  for (int pwr_idx = 0; pwr_idx < 11; pwr_idx++) {
    float f3 = pgm_read_float_near(&flow3_2d_lookup_banked[pwr_idx][v3_idx]);
    float f4 = pgm_read_float_near(&flow4_2d_lookup_banked[pwr_idx][v4_idx]);

    float err = abs(f3 - q3_des) + abs(f4 - q4_des);
    if (err < min_combined_error) {
      min_combined_error = err;
      best_idx = pwr_idx;
    }
  }

  // Interpolation with next index
  if (best_idx < 10) {
    float f3_lo = pgm_read_float_near(&flow3_2d_lookup_banked[best_idx][v3_idx]);
    float f4_lo = pgm_read_float_near(&flow4_2d_lookup_banked[best_idx][v4_idx]);
    float f3_hi = pgm_read_float_near(&flow3_2d_lookup_banked[best_idx + 1][v3_idx]);
    float f4_hi = pgm_read_float_near(&flow4_2d_lookup_banked[best_idx + 1][v4_idx]);

    float q3_diff = q3_des - f3_lo;
    float q4_diff = q4_des - f4_lo;
    float denom3 = f3_hi - f3_lo;
    float denom4 = f4_hi - f4_lo;

    float w3 = (denom3 != 0) ? q3_diff / denom3 : 0;
    float w4 = (denom4 != 0) ? q4_diff / denom4 : 0;

    float interp_idx = best_idx + (w3 + w4) / 2.0;

    // Clamp index to [0, 10]
    if (interp_idx > 10) interp_idx = 10;
    if (interp_idx < 0) interp_idx = 0;

    // Power index to % mapping: 0 → 30%, 1 → 32%, ..., 10 → 50%
    return int(2 * interp_idx + 30);  // 2% step size
  }

  return 30 + best_idx * 2;
}
*/

int getPumpPower_indiv(int nozzle, float q_des) {
  switch (nozzle) {
    case 1:
      lookUpTableFlow = lookUpTableFlow1;
      lookUpTablePwr = lookUpTablePwr1;
      break;
    case 2:
      lookUpTableFlow = lookUpTableFlow2;
      lookUpTablePwr = lookUpTablePwr2;
      break;
    case 3:
      lookUpTableFlow = lookUpTableFlow3;
      lookUpTablePwr = lookUpTablePwr3;
      break;
    case 4:
      lookUpTableFlow = lookUpTableFlow4;
      lookUpTablePwr = lookUpTablePwr4;
      break;
  }
  // 7 is size of look up tables
  for (int i = 0; i <= 7 - 1; i++) {
    if (q_des >= lookUpTableFlow[i]) {
      int index = i; 
      int nxt_index = i+1;
      float weight_fxn = (q_des - lookUpTableFlow[index]) / (lookUpTableFlow[nxt_index] - lookUpTableFlow[index]);
      calc_u_base = lookUpTablePwr[index] + ((weight_fxn)*(lookUpTablePwr[nxt_index] - lookUpTablePwr[index])); 
    }
  }

  if (calc_u_base > 60)
    calc_u_base = 60;
  if (calc_u_base < 0)
    calc_u_base = 0;

  return calc_u_base; 
}

float computePIPump(int i, float dfr, float flow_rate) {
  eFlow_pump[i] = dfr - flow_rate;
  
  //if (abs(eFlow_pump[i]) < 10) 
    //error_i[i] += eFlow_pump[i]; 

  error_i[i] += eFlow_pump[i]; 

  int_pump[i] = ki_pump[i] * error_i[i];
  p_pump[i] = kp_pump[i] * eFlow_pump[i];

  // integral windup
  float z = 6.0;
  if (int_pump[i] > z)
    int_pump[i] = z;
  if (int_pump[i] < -z)
    int_pump[i] = -z;

  if (p_pump[i] > 6.0)
    p_pump[i] = 6.0;
  if(p_pump[i] < -6.0)
    p_pump[i] = -6.0;

  u_PI[i] = p_pump[i] + int_pump[i];

  if (u_PI[i] > 20)
    u_PI[i] = 20;
  if (u_PI[i] < -20)
    u_PI[i] = -20; 
  if (abs(u_PI[i]) < 0.5) 
    u_PI[i] = 0;

  return u_PI[i];
}

void updatePumpPower() {
  if (isBanked) {
    kp_pump[0] = 0.0;     kp_pump[1] = 0.1; 
    kp_pump[2] = 0.0;     kp_pump[3] = 0.1;

    ki_pump[0] = 0.0;     ki_pump[1] = 0.05; 
    ki_pump[2] = 0.0;     ki_pump[3] = 0.05; 

    u_base[0] = 0;    // pump is off in banked system
    u_base[1] = getPumpPower_L(valve_pos[0], valve_pos[1], dfr[0], dfr[1]);
    u_base[2] = 0;    // pump is off in banked system
    u_base[3] = getPumpPower_R(valve_pos[2], valve_pos[3], dfr[2], dfr[3]);

    u_PI[0] = 0;      // pump is off in banked system
    u_PI[1] = computePIPump(1, dfr_L, flow_rate_left);
    u_PI[2] = 0;      // pump is off in banked system
    u_PI[3] = computePIPump(3, dfr_R, flow_rate_right);
    
    if (u_base[1] > 60)
      u_base[1] = 60;
    if (u_base[1] < 0)
      u_base[1] = 0;
    if (u_base[3] > 60)
      u_base[3] = 60;
    if (u_base[3] < 0)
      u_base[3] = 0;

    pump_pwr[0] = 0;                        // not used
    pump_pwr[1] = u_base[1]; //+ u_PI[1];      // Pump 2 -> Nozzles 1&2
    pump_pwr[2] = 0;                        // not used
    pump_pwr[3] = u_base[3]; //+ u_PI[3];      // Pump 4 -> Nozzles 3&4

    if (pump_pwr[1] > 60)
      pump_pwr[1] = 60;
    if (pump_pwr[3] < 0)
      pump_pwr[3] = 0;
  }
  else {
    kp_pump[0] = 0.07; 
    kp_pump[1] = 0.07; 
    kp_pump[2] = 0.07; 
    kp_pump[3] = 0.07;

    ki_pump[0] = 0.03;
    ki_pump[1] = 0.03;
    ki_pump[2] = 0.03;
    ki_pump[3] = 0.03; 

    // Each nozzle has its own pump 
    for (int i = 0; i < 4; i++) {
      u_base[i] = getPumpPower_indiv(i, dfr[i]);
      u_PI[i] = computePIPump(i, dfr[i], flow_rate[i]); 

      pump_pwr[i] = u_base[i] + u_PI[i];

      if (pump_pwr[i] > 60)
        pump_pwr[i] = 60;
      if (pump_pwr[i] < 0)
        pump_pwr[i] = 0; 
      
      if (valve_pos[i] < 10 || dfr[i] == 0) {
        pump_pwr[i] = 0; 
      }
    }
  } 
}

void setValveRatios_L(float q1_des, float q2_des) {
  float ratio;
  float max_pos = 100.0;
  float min_pos = 0.0;

  if (q2_des != 0) {
    ratio = q1_des / q2_des;
  } else {
    ratio = 1.0; // fallback to equal if denominator is 0
  }

  if (ratio > 1.0) {
    // set valve1 all the way open & turn off valve1 PI control
    valve_pos[0] = max_pos; 
    kp_nozzle[0] = 0.0;
    ki_nozzle[0] = 0.0;

    // set valve2 position based on ratio & turn on valve2 PI control
    valve_pos[1] = valve_pos[0] / ratio;    
    kp_nozzle[1] = 0.001;
    ki_nozzle[1] = 0.01;

  } else {
    // set valve2 all the way open & turn off valve2 PI control
    valve_pos[1] = max_pos;
    kp_nozzle[1] = 0.0;
    ki_nozzle[1] = 0.0;

    // set valve1 position based on ratio & turn on valve1 PI control 
    valve_pos[0] = valve_pos[1] * ratio;
    kp_nozzle[0] = 0.001;
    ki_nozzle[0] = 0.01;
    
  }

  // Clamp both valves
  if (valve_pos[0] < min_pos) valve_pos[0] = min_pos;
  if (valve_pos[1] < min_pos) valve_pos[1] = min_pos;
}

void setValveRatios_R(float q3_des, float q4_des) {
  float ratio;
  float max_pos = 100.0;
  float min_pos = 0.0;

  if (q4_des != 0) {
    ratio = q3_des / q4_des;
  } else {
    ratio = 1.0; // fallback to equal if denominator is 0
  }

  if (ratio > 1.0) {
    // set valve3 all the way open & turn off valve3 PI control
    valve_pos[2] = max_pos;
    kp_nozzle[2] = 0.0;
    ki_nozzle[2] = 0.0;

    // set valve4 position based on ratio & turn on valve4 PI control
    valve_pos[3] = valve_pos[2] / ratio;
    kp_nozzle[3] = 0.008; 
    ki_nozzle[3] = 0.01;

  } else {
    // set valve4 all the way open & turn off valve4 PI control
    valve_pos[3] = max_pos;
    kp_nozzle[3] = 0.0;
    ki_nozzle[3] = 0.0;

    // set valve3 position based on ratio & turn on valve4 PI control
    valve_pos[2] = valve_pos[3] * ratio;
    kp_nozzle[2] = 0.003;
    ki_nozzle[2] = 0.01;
    
  }

  // Clamp both valves
  if (valve_pos[2] < min_pos) valve_pos[2] = min_pos;
  if (valve_pos[3] < min_pos) valve_pos[3] = min_pos;
}

void setup() {
  Serial.begin(9600);     // debug output and ROS communication
  Serial1.begin(9600);    // flow sensor readings from arduino uno
  Serial2.begin(115200);  // roboteq for left bank
  Serial3.begin(115200);  // roboteq for right bank
  delay(100);

  cloopTime = millis();
  cloopTime2 = millis();

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  // pinMode(pulsePin, OUTPUT); // for debug to manually trigger pulses 

  // open valves partially to allow flow thru at the beginning 
  set_valve(1, 0);
  set_valve(2, 0);
  set_valve(3, 0);
  set_valve(4, 0);

  for (int i = 0; i < 4; i++) {
    u_PI[i] = 0;
  }
}

int cal_pump_pwr(int r)
{
  int x = r;
  int a1 = 50; int a2 = 200;
  int b1 = 21; int b2 = 62;

  float f1 = float(x-50)/float(a2-a1);
  float yy = b1 + (float(b2-b1))*f1;

  return int(yy);
}

void loop() {
  get_flow_rates();

  flow_rate_left = flow_rate[0] + flow_rate[1];
  flow_rate_right = flow_rate[2] + flow_rate[3];

  currentT = millis();
  if (currentT >= (cloopT + 500))
  {
    cloopT = currentT; // Updates cloopT

    //pump_pwr[0] = maf_pump1(pump_pwr_uf[0]);

    ctrl_pump(1, pump_pwr[0]); ctrl_pump(2, pump_pwr[1]);
    ctrl_pump(3, pump_pwr[2]); ctrl_pump(4, pump_pwr[3]);
    set_valve(1, valve_pos[0]); set_valve(2, valve_pos[1]);
    set_valve(3, valve_pos[2]); set_valve(4, valve_pos[3]);

    //Serial.println("testing");

    //debug_disp();
    //send2ros();
    send2Matlab();
  }
/*
 for matlab logging, uncomment this and comment out the next if statement
  if (Serial.available())
  {
    dfr[0] = Serial.parseInt();
    dfr[1] = Serial.parseInt();
    dfr[2] = Serial.parseInt();
    dfr[3] = Serial.parseInt();
    dfr_L = dfr[0] + dfr[1];
    dfr_R = dfr[2] + dfr[3];
  }*/

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // remove any leading/trailing whitespace
    int idx = 0;

    // Parse up to 4 integers from the string
    while (input.length() > 0 && idx < 4) {
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx == -1) {
        dfr[idx++] = input.toInt();
        break;
      } else {
        dfr[idx++] = input.substring(0, spaceIdx).toInt();
        input = input.substring(spaceIdx + 1);
      }
    }

    // Update combined flows
    dfr_L = dfr[0] + dfr[1];
    dfr_R = dfr[2] + dfr[3];
  }
  /*
  // --- Zone ID Input ---
  if (Serial.available())
  {
    int xx = Serial.parseInt();
    //zone_id = Serial.parseInt();
    if (xx != 0) zone_id = xx;
    //Serial.println(zone_id);
  }
  */

  // --- Zero Flow Case ---
  if (dfr_L == 0 && dfr_R == 0) {
    u_base[1] = 0;
    u_PI[1] = 0;
    u_base[3] = 0;
    u_PI[3] = 0;
    pump_pwr[0] = 0;
    pump_pwr[1] = 0;
    pump_pwr[2] = 0;
    pump_pwr[3] = 0;

    // Optional: Also set valve positions to 0 or minimum
    valve_pos[0] = 0;
    valve_pos[1] = 0;
    valve_pos[2] = 0;
    valve_pos[3] = 0;

    return; // skip rest of loop
  }

  updatePumpPower();

  // Valve PI Control
  if (isBanked) {
    setValveRatios_L(dfr[0], dfr[1]);
    setValveRatios_R(dfr[2], dfr[3]);

    for (int i=0; i<4; i++) {
    eFlow[i] = dfr[i] - flow_rate[i]; // flow error

    if (abs(eFlow[i])<30)
      eFlow_i[i] += eFlow[i];

    nozzle_PI[i] = (kp_nozzle[i] * eFlow[i]) + (ki_nozzle[i] * eFlow_i[i]);

    valve_pos[i] += nozzle_PI[i]; // pi control

    if (valve_pos[i] > 100) valve_pos[i] = 100;
    if (valve_pos[i] < 10) valve_pos[i] = 10; 
    
    if (dfr[i] == 0){
      valve_pos[i] -= 5;
      if (valve_pos[i] < 0) valve_pos[i] = 0;
    }
    if(dfr[i] > 0 && valve_pos[i] < 10) {
      valve_pos[i] = 10;
    }
  }
  }
  else {
    // for 1-to-1 system, keep valves open
    valve_pos[0] = 100;
    valve_pos[1] = 100;
    valve_pos[2] = 100;
    valve_pos[3] = 100;
  }
 
}
