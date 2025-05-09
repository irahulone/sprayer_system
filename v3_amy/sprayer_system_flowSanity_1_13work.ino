// This code takes user input of valve number (1-4) valve position (0-100) pump choice (1 or 2) pump power (0-100)
// Serial Monitor User Input will look something like this: 2 50 1 70
// where the valve 2 is set to halfway open and pump 1 is set to 70% power

#include <Arduino.h>
#include <SPI.h>

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

// Define Slave Select pins for the LS7366R Encoders (connected to flow sensors)

const int slaveSelectEnc1 = 40;
const int slaveSelectEnc2 = 41;
const int slaveSelectEnc3 = 42;
const int slaveSelectEnc4 = 43;

// Pump control variables
float pump_pwr[2] = { 0.0, 0.0 };             // Pump Power Value that gets passed into the PUMPS
float valve_pos[4] = { 0.0, 0.0, 0.0, 0.0 };  // Valve position for each valve
float flow_rate_uf[4] = { 0.0, 0.0, 0.0, 0.0 };
float flow_rate[4] = { 0.0, 0.0, 0.0, 0.0 };

// This holds the current encoder count
long lastCount_1 = 0;
long lastCount_2 = 0;
long lastCount_3 = 0;
long lastCount_4 = 0;
unsigned long lastTime = 0;
long pulses[4] = { 0, 0, 0, 0 };

unsigned int l_hour1, l_hour2, l_hour3, l_hour4; // Calculated liters/hour
unsigned long currentTime; 
unsigned long cloopTime = 0; // to measure flow rate

const int window_size = 400;  // define the window size
int flow_s0_ar[window_size];
int flow_s1_ar[window_size];
int flow_s2_ar[window_size];
int flow_s3_ar[window_size];


// Function to initialize flow sensors
void initEncoder() {
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  pinMode(slaveSelectEnc3, OUTPUT);
  pinMode(slaveSelectEnc4, OUTPUT);

  digitalWrite(slaveSelectEnc1, HIGH);
  digitalWrite(slaveSelectEnc2, HIGH);
  digitalWrite(slaveSelectEnc3, HIGH);
  digitalWrite(slaveSelectEnc4, HIGH);

  SPI.begin();

  for (int i = 1; i <= 4; i++) {
    int slaveSelect = (i == 1) ? slaveSelectEnc1 : (i == 2) ? slaveSelectEnc2 : (i == 3) ? slaveSelectEnc3 : slaveSelectEnc4;
    digitalWrite(slaveSelect, LOW);
    SPI.transfer(0x88);
    SPI.transfer(0x00);
    digitalWrite(slaveSelect, HIGH);
  }
}

// Function to count pulses from each flow sensor
long readEncoder(int encoder) {
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;
  int slaveSelect = (encoder == 1) ? slaveSelectEnc1 : (encoder == 2) ? slaveSelectEnc2 : (encoder == 3) ? slaveSelectEnc3 : slaveSelectEnc4;

  digitalWrite(slaveSelect, LOW);
  SPI.transfer(0x60);
  count_1 = SPI.transfer(0x00);
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);
  digitalWrite(slaveSelect, HIGH);

  count_value = (count_1 << 24) | (count_2 << 16) | (count_3 << 8) | count_4;
  return count_value;
}

// Function to clear flow sensor pulse count
void clearEncoderCount() {
  for (int i = 1; i <= 4; i++) {
    int slaveSelect = (i == 1) ? slaveSelectEnc1 : (i == 2) ? slaveSelectEnc2 : (i == 3) ? slaveSelectEnc3 : slaveSelectEnc4;
    digitalWrite(slaveSelect, LOW);
    SPI.transfer(0x98);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    digitalWrite(slaveSelect, HIGH);

    delayMicroseconds(100);

    digitalWrite(slaveSelect, LOW);
    SPI.transfer(0xE0);
    digitalWrite(slaveSelect, HIGH);
  }
}

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

  if (ch == 1) {
    Serial3.print("!G 1 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }

  if (ch == 2) {
    Serial3.print("!G 2 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }
}

void debug_disp() {
  Serial.print("vlv"); Serial.print("\t");
  Serial.print(valve_pos[0], 0); Serial.print("\t"); Serial.print(valve_pos[1], 0); Serial.print("\t");
  Serial.print(valve_pos[2], 0); Serial.print("\t"); Serial.print(valve_pos[3], 0); Serial.print("\t");
  Serial.println();

  Serial.print("flo"); Serial.print(" L/h\t");
  Serial.print(flow_rate[0], 1); Serial.print("\t"); Serial.print(flow_rate[1], 1); Serial.print("\t");
  Serial.print(flow_rate[2], 1); Serial.print("\t"); Serial.print(flow_rate[3], 1); Serial.print("\t");
  Serial.println();

  Serial.print("pmp"); Serial.print("\t");
  Serial.print(pump_pwr[0], 0); Serial.print("\t"); Serial.print(pump_pwr[1], 0); Serial.print("\t");
  Serial.println();
  //Serial.println();

  Serial.print("puls"); Serial.print("\t");
  Serial.print(pulses[0], 1); Serial.print("\t"); Serial.print(pulses[1], 1);Serial.print("\t");
  Serial.print(pulses[2], 1); Serial.print("\t"); Serial.print(pulses[3], 1); Serial.print("\t");
  Serial.println();
  Serial.println();
}

void get_flow_rates() {
  
  currentTime = millis();
  //unsigned long timeInterval = currentTime - lastTime;
  
  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime; // Update cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min 
    // (PUlse frequency x 60 min) / 7.5Q = flowrate in L/hour

    // Read encoder counts
    long currentCount_1 = readEncoder(1);
    long currentCount_2 = readEncoder(2);
    long currentCount_3 = readEncoder(3);
    long currentCount_4 = readEncoder(4);

    // Calculate pulses
    pulses[0] = currentCount_1 - lastCount_1;
    pulses[1] = currentCount_2 - lastCount_2;
    pulses[2] = currentCount_3 - lastCount_3;
    pulses[3] = currentCount_4 - lastCount_4;

    // Calculate flow freqeuncies and flow rates
    float flowfrequency1 = (float)pulses[0];
    float flowfrequency2 = (float)pulses[1];
    float flowfrequency3 = (float)pulses[2];
    float flowfrequency4 = (float)pulses[3];

    l_hour1 = (flowfrequency1 * 60.0 / 7.5);
    l_hour2 = (flowfrequency2 * 60.0 / 7.5);
    l_hour3 = (flowfrequency3 * 60.0 / 7.5);
    l_hour4 = (flowfrequency4 * 60.0 / 7.5);

    // Store unfiltered flow rates
    flow_rate[0] = l_hour1;
    flow_rate[1] = l_hour2;
    flow_rate[2] = l_hour3;
    flow_rate[3] = l_hour4;

    // Update for the next interval
    lastCount_1 = currentCount_1;
    lastCount_2 = currentCount_2;
    lastCount_3 = currentCount_3;
    lastCount_4 = currentCount_4;

    // Debug output
    debug_disp();
    
    // Reset Counters
    flowfrequency1 = 0;
    flowfrequency2 = 0;
    flowfrequency3 = 0;
    flowfrequency4 = 0;

  }
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(115260);
  delay(100);
  initEncoder();
  Serial.println("Encoder Initialized...");
  clearEncoderCount();
  Serial.println("Encoder Count Cleared...");
  lastTime = millis();

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  set_valve(1, 0);
  set_valve(2, 0);
  set_valve(3, 0);
  set_valve(4, 0);
}

void loop() {
  get_flow_rates();
  
  ctrl_pump(1, pump_pwr[0]);
  ctrl_pump(2, pump_pwr[1]);
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

    if (pumpNum == 1) {
      pump_pwr[0] = pumpPwr;
    }
    if (pumpNum == 2) {
      pump_pwr[1] = pumpPwr;
    }
  }
}
