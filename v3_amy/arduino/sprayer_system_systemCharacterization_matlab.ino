// This code takes user input of valve number (1-4) valve position (0-100) pump choice (1 or 2) pump power (0-100)
// Serial Monitor User Input will look something like this: 2 50 1 70
// where the valve 2 is set to halfway open and pump 1 is set to 70% power

#include <Arduino.h>

// Define the valve control pins for M1, M2, M3, M4 (positive terminals) from DF robot 4ch driver shield, DRI0039
// https://wiki.dfrobot.com/Quad_Motor_Driver_Shield_for_Arduino_SKU_DRI0039
// M1-M4 are valve PWM
#define M1 4
#define M2 12
#define M3 8
#define M4 7
// E1=E4 are valve dir
#define E1 3
#define E2 11 
#define E3 5
#define E4 6


// Pump control variables
float pump_pwr[4] = { 0.0, 0.0, 0.0, 0.0 };             // Pump Power Value that gets passed into the PUMPS
float valve_pos[4] = { 0.0, 0.0, 0.0, 0.0 };            // Valve position for each valve
float flow_rate_uf[4] = { 0.0, 0.0, 0.0, 0.0 };
float flow_rate[4] = { 0.0, 0.0, 0.0, 0.0 };
long pulses[4] = { 0, 0, 0, 0 };

// true = banked system (2 pumps), false = 1-to-1 system (4 pumps)
bool isBanked = false; 


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
  //Serial.print(voltage);
  //Serial.print(" V (PWM value: ");
  //Serial.print(pwmValue);
  //Serial.println(")");
}

void ctrl_pump(int ch = 0, int val = 0) {
  int min_val_pos = 10;
  int max_per = 60;
  if (val < 0) val = 0;
  else if (val > max_per) val = max_per;

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

  int v = map(val, 0, 100, 0, 1000);

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


void get_flow_rates() {
  if (Serial1.available()) {
    flow_rate[0] = Serial1.parseFloat();
    flow_rate[1] = Serial1.parseFloat();
    flow_rate[2] = Serial1.parseFloat();
    flow_rate[3] = Serial1.parseFloat();

    // pulses[0] = Serial1.parseInt();
    // pulses[1] = Serial1.parseInt();
    // pulses[2] = Serial1.parseInt();
    // pulses[3] = Serial1.parseInt();
  }
}


void debug_disp() {
    Serial.print(pump_pwr[0]); Serial.print(",");
    Serial.print(valve_pos[0]); Serial.print(",");
    Serial.print(flow_rate[0]); Serial.print(",");

    Serial.print(pump_pwr[1]); Serial.print(",");
    Serial.print(valve_pos[1]); Serial.print(",");
    Serial.print(flow_rate[1]); Serial.print(",");
    
    Serial.print(pump_pwr[2]); Serial.print(",");
    Serial.print(valve_pos[2]); Serial.print(",");
    Serial.print(flow_rate[2]); Serial.print(",");
    
    Serial.print(pump_pwr[3]); Serial.print(",");
    Serial.print(valve_pos[3]); Serial.print(",");
    Serial.print(flow_rate[3]); Serial.println(); 
}


void setup() {
  Serial.begin(9600);       // debug disp
  Serial1.begin(9600);      // flow sensor readings for arduino uno
  Serial2.begin(115200);    // roboteq for left bank
  Serial3.begin(115200);    // roboteq for right bank
  delay(100);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  set_valve(1, 0);
  set_valve(2, 0);
  set_valve(3, 0);
  set_valve(4, 0);

  pump_pwr[0] = 0;
  pump_pwr[1] = 0;
  pump_pwr[2] = 0;
  pump_pwr[3] = 0;
}

void loop() {
  get_flow_rates();
  delay(500);
  
  ctrl_pump(1, pump_pwr[0]);
  ctrl_pump(2, pump_pwr[1]);
  ctrl_pump(3, pump_pwr[2]);
  ctrl_pump(4, pump_pwr[3]);

  set_valve(1, valve_pos[0]);
  set_valve(2, valve_pos[1]);
  set_valve(3, valve_pos[2]);
  set_valve(4, valve_pos[3]);

  if (Serial.available()) {    
    int valveNum = Serial.parseInt();
    int valvePos = Serial.parseInt();
    int pumpNum = Serial.parseInt();
    int pumpPwr = Serial.parseInt();

    if (valveNum == 1) {
      valve_pos[0] = valvePos;
    }
    if (valveNum == 2) {
      valve_pos[1] = valvePos;
    }
    if (valveNum == 3) {
      valve_pos[2] = valvePos;
    }
    if (valveNum == 4) {
      valve_pos[3] = valvePos;
    }

    if (isBanked) {
      if (pumpNum == 1) {
        pump_pwr[0] = 0;
        pump_pwr[1] = float(pumpPwr);
      }
      if (pumpNum == 2) {
        pump_pwr[2] = 0;
        pump_pwr[3] = float(pumpPwr);
      }
    }
    else {
      if (pumpNum == 1) {
        pump_pwr[0] = float(pumpPwr);
      }
      if (pumpNum == 2) {
        pump_pwr[1] = float(pumpPwr);
      }
      if (pumpNum == 3) {
        pump_pwr[2] = float(pumpPwr);
      }
      if (pumpNum == 4) {
        pump_pwr[3] = float(pumpPwr);
      }
    }
    
  }
  debug_disp();
}
