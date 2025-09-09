// This code uses digital pins to constantly read the encoder signal 
// If encoder switches from LOW to HIGH it adds 1 to the counter
// After 1 min, total # of counts is used to calculate flow --> counter reset 

#include <Arduino.h>

const int flowSensor[4] = {10, 11, 12, 13};

float flow_rate_uf[4] = { 0.0, 0.0, 0.0, 0.0 };
float flow_rate[4] = { 0.0, 0.0, 0.0, 0.0 };

// This holds the current encoder count
unsigned long lastTime = 0;
unsigned long count[4] = {0, 0, 0, 0};
bool lastState[4] = {LOW, LOW, LOW, LOW};
long pulses[4] = { 0, 0, 0, 0 };

unsigned int l_hour1, l_hour2, l_hour3, l_hour4;    // Calculated liters/hour
unsigned long currentTime;                          // current time for flow measurement 
unsigned long cloopTime = 0;                        // to measure flow rate
unsigned long cloopTime2;                           // to compute moving average
unsigned long currentT;                             // current time for moving average filter

const int window_size = 10;                        // define the window size
int flow_s0_ar[window_size];
int flow_s1_ar[window_size];
int flow_s2_ar[window_size];
int flow_s3_ar[window_size];


void countPulses() {
  for (int i = 0; i < 4; i++) {
    bool currentState = digitalRead(flowSensor[i]);
    if (lastState[i] == HIGH && currentState == LOW) {
      count[i]++;
    }
    lastState[i] = currentState;
  }
}


float maf_s0(float x)  // moving average filters
{
  int p = window_size;
  flow_s0_ar[p] = x;
  float a = 0;
  for (int i = 1; i <= p; i++)
    a += flow_s0_ar[i];
  a = a / p;
  for (int i = 2; i <= p; i++)
    flow_s0_ar[i - 1] = flow_s0_ar[i];
  return a;
}

float maf_s1(float x)  // moving average filters
{
  int p = window_size;
  flow_s1_ar[p] = x;
  float a = 0;
  for (int i = 1; i <= p; i++)
    a += flow_s1_ar[i];
  a = a / p;
  for (int i = 2; i <= p; i++)
    flow_s1_ar[i - 1] = flow_s1_ar[i];
  return a;
}

float maf_s2(float x)  // moving average filters
{
  int p = window_size;
  flow_s2_ar[p] = x;
  float a = 0;
  for (int i = 1; i <= p; i++)
    a += flow_s2_ar[i];
  a = a / p;
  for (int i = 2; i <= p; i++)
    flow_s2_ar[i - 1] = flow_s2_ar[i];
  return a;
}

float maf_s3(float x)  // moving average filters
{
  int p = window_size;
  flow_s3_ar[p] = x;
  float a = 0;
  for (int i = 1; i <= p; i++)
    a += flow_s3_ar[i];
  a = a / p;
  for (int i = 2; i <= p; i++)
    flow_s3_ar[i - 1] = flow_s3_ar[i];
  return a;
}



void debug_disp() {
  Serial.print("flo"); Serial.print(" L/h\t");
  Serial.print(flow_rate[0], 1); Serial.print("\t"); Serial.print(flow_rate[1], 1); Serial.print("\t");
  Serial.print(flow_rate[2], 1); Serial.print("\t"); Serial.print(flow_rate[3], 1); Serial.print("\t");
  Serial.println();

  Serial.print("puls"); Serial.print("\t");
  Serial.print(pulses[0], 1); Serial.print("\t"); Serial.print(pulses[1], 1);Serial.print("\t");
  Serial.print(pulses[2], 1); Serial.print("\t"); Serial.print(pulses[3], 1); Serial.print("\t");
  Serial.println();

  Serial.print("count"); Serial.print("\t");
  Serial.print(count[0], 1); Serial.print("\t"); Serial.print(count[1], 1);Serial.print("\t");
  Serial.print(count[2], 1); Serial.print("\t"); Serial.print(count[3], 1); Serial.print("\t");
  Serial.println();
  Serial.println();
}

void print_data() {
  Serial.print(flow_rate[0], 1);    Serial.print("\t");
  Serial.print(flow_rate[1], 1);    Serial.print("\t");
  Serial.print(flow_rate[2], 1);    Serial.print("\t");
  Serial.print(flow_rate[3], 1);    Serial.print("\t");
  //Serial.print(pulses[0]);          Serial.print("\t");
  //Serial.print(pulses[1]);          Serial.print("\t");
  //Serial.print(pulses[2]);          Serial.print("\t");
  //Serial.print(pulses[3]);
  Serial.println();
}

void meas_flow_rates() {
  
  currentTime = millis();
  //unsigned long timeInterval = currentTime - lastTime;
  
  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime; // Update cloopTime
    
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min 
    // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour

    // Pulse frequency (Hz) = 23Q, Q is flow rate in L/min 
    // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour


    for (int i = 0; i < 4; i++) {
      pulses[i] = count[i];
      count[i] = 0;

      float flowfreq = pulses[i];
      flow_rate_uf[i] = (flowfreq * 60.0 / 7.5); 
      // flow_rate_uf[i] = (flowfreq * 60.0 / 23);
    }
    // Debug output
    //debug_disp();
    print_data();
  }
  if (currentTime >= (cloopTime2 + 100))
  {
    cloopTime2 = currentTime; 
    flow_rate[0] = maf_s0(flow_rate_uf[0]);
    flow_rate[1] = maf_s1(flow_rate_uf[1]);
    flow_rate[2] = maf_s2(flow_rate_uf[2]);
    flow_rate[3] = maf_s3(flow_rate_uf[3]);
  } 
}


void setup() {
  Serial.begin(9600);
  delay(100);
  lastTime = millis();
  cloopTime = millis();
  cloopTime2 = millis();

  for (int i = 0; i < 4; i++) {
    pinMode(flowSensor[i], INPUT);
    lastState[i] = digitalRead(flowSensor[i]);
  }
}

void loop() {
  countPulses();       // check for rising edges constantly
  meas_flow_rates();   // once per second, compute flow 
}
