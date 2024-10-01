
// Hardware - Arduino Mega 2560


float pump_pwr[2]  = {0.0, 0.0};
float valve_pos[4] = {0.0, 0.0, 0.0, 0.0};
float flow_rate_uf[4] = {0.0, 0.0, 0.0, 0.0};
float flow_rate[4] = {0.0, 0.0, 0.0, 0.0};

const int window_size = 400;  // define the window size.
int flow_s0_ar[window_size];
int flow_s1_ar[window_size];
int flow_s2_ar[window_size];
int flow_s3_ar[window_size];

// pins specific to DF robot 4ch driver shield, DRI0039
// https://wiki.dfrobot.com/Quad_Motor_Driver_Shield_for_Arduino_SKU_DRI0039
const int E1 = 3; // valve1 pwm
const int E2 = 11;// valve2 pwm
const int E3 = 5; // valve3 pwm
const int E4 = 6; // valve4 pwm
const int M1 = 4; // valve1 dir
const int M2 = 12;// valve2 dir
const int M3 = 8; // valve3 dir
const int M4 = 7; // valve4 dir



volatile int flow_frequency18, flow_frequency19, flow_frequency20, flow_frequency21 ; // Measures flow sensor pulses
unsigned int l_hour18, l_hour19, l_hour20, l_hour21; // Calculated litres/hour
const byte flowsensor18  =  18;  // Flow Sensor Input 1
const byte flowsensor19  =  19;  // Flow Sensor Input 2
const byte flowsensor20  =  20;  // Flow Sensor Input 3
const byte flowsensor21  =  21;  // Flow Sensor Input 4
unsigned long currentTime;
unsigned long cloopTime;
unsigned long cloopTime2;

void flow18 () // Interrupt function
{
  flow_frequency18++;
}
void flow19 () // Interrupt function
{
  flow_frequency19++;
}
void flow20 () // Interrupt function
{
  flow_frequency20++;
}
void flow21 () // Interrupt function
{
  flow_frequency21++;
}

void set_valve(int i = 0, int x = 0)
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M3,LOW);
  digitalWrite(M4,HIGH);

  if (x < 0) x = 0;
  else if (x >= 100) x = 100;
  int v = map(x, 0, 100, 0, 255);
  
  if (i == 1) analogWrite(E1, v);
  if (i == 2) analogWrite(E2, v);
  if (i == 3) analogWrite(E3, v);
  if (i == 4) analogWrite(E4, v);
}

void ctrl_pump(int ch = 0, int val = 0)
{
  int min_val_pos = 20;
  if (ch == 1)
  {
    if(valve_pos[0] < min_val_pos && valve_pos[1] < min_val_pos)
    val = 0;  
  }
  if (ch == 2)
  {
    if(valve_pos[2] < min_val_pos && valve_pos[3] < min_val_pos)
    val = 0;  
  }
  
  
  int max_per = 60;
  if(val < 0) val = 0;
  else if(val > max_per) val = max_per;
  int v = map(val, 0, 100, 0, 1000);

  if (ch==1)
  {
    Serial3.print("!G 1 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }

  if (ch==2)
  {
    Serial3.print("!G 2 ");
    Serial3.print(String(v));
    Serial3.print("_\r");
  }
}

void debug_disp()
{
  Serial.print("vlv"); Serial.print("\t");
  Serial.print(valve_pos[0],0); Serial.print("\t"); Serial.print(valve_pos[1],0); Serial.print("\t");
  Serial.print(valve_pos[2],0); Serial.print("\t"); Serial.print(valve_pos[3],0); Serial.print("\t");
  Serial.println();

  Serial.print("flo"); Serial.print("\t");
  Serial.print(flow_rate[0], 1); Serial.print("\t"); Serial.print(flow_rate[1], 1); Serial.print("\t");
  Serial.print(flow_rate[2], 1); Serial.print("\t"); Serial.print(flow_rate[3], 1); Serial.print("\t");
  Serial.println();
  
  Serial.print("pmp"); Serial.print("\t");
  Serial.print(pump_pwr[0], 0); Serial.print("\t"); Serial.print(pump_pwr[1], 0); Serial.print("\t");
  Serial.println();
  Serial.println();
}

float maf_s0(float x)
{
  int p = window_size;
  flow_s0_ar[p] = x;
  float a = 0;
  for(int i = 1; i <= p; i++)
    a += flow_s0_ar[i];
  a = a/p;
  for(int i = 2; i <= p; i++)
    flow_s0_ar[i-1] = flow_s0_ar[i];
  return a;
}
float maf_s1(float x)
{
  int p = window_size;
  flow_s1_ar[p] = x;
  float a = 0;
  for(int i = 1; i <= p; i++)
    a += flow_s1_ar[i];
  a = a/p;
  for(int i = 2; i <= p; i++)
    flow_s1_ar[i-1] = flow_s1_ar[i];
  return a;
}
float maf_s2(float x)
{
  int p = window_size;
  flow_s2_ar[p] = x;
  float a = 0;
  for(int i = 1; i <= p; i++)
    a += flow_s2_ar[i];
  a = a/p;
  for(int i = 2; i <= p; i++)
    flow_s2_ar[i-1] = flow_s2_ar[i];
  return a;
}
float maf_s3(float x)
{
  int p = window_size;
  flow_s3_ar[p] = x;
  float a = 0;
  for(int i = 1; i <= p; i++)
    a += flow_s3_ar[i];
  a = a/p;
  for(int i = 2; i <= p; i++)
    flow_s3_ar[i-1] = flow_s3_ar[i];
  return a;
}


void get_flow_rates()
{
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 23Q, Q is flow rate in L/min.

    l_hour18 = ((flow_frequency18 * 60 / 7.5) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency18 = 0; // Reset Counter
    flow_rate_uf[0] = l_hour18;
    
    l_hour19 = ((flow_frequency19 * 60 / 7.5) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency19 = 0; // Reset Counter
    flow_rate_uf[1] = l_hour19;
   
    l_hour20 = ((flow_frequency20 * 60 / 7.5) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency20 = 0; // Reset Counter
    flow_rate_uf[2] = l_hour20;
    
    l_hour21 = ((flow_frequency21 * 60 / 7.5) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency21 = 0; // Reset Counter
    flow_rate_uf[3] = l_hour21;  

      debug_disp();
  }

  // moving average filter
  if (currentTime >= (cloopTime2 + 1000))
  {
   flow_rate[0] = maf_s0(flow_rate_uf[0]);
   flow_rate[1] = maf_s1(flow_rate_uf[1]);
   flow_rate[2] = maf_s2(flow_rate_uf[2]);
   flow_rate[3] = maf_s3(flow_rate_uf[3]);
   
  }

}



void setup()
{
  pinMode(flowsensor18, INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor19, INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor20, INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor21, INPUT_PULLUP);  // Optional Internal Pull-Up
  
  attachInterrupt(digitalPinToInterrupt(flowsensor18), flow18,  RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor19), flow19,  RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor20), flow20, RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor21), flow21, RISING); // Setup Interrupt
  currentTime = millis();
  cloopTime = currentTime;


  Serial.begin(9600);
  Serial3.begin(115260);  // roboteq controller

  // declare valve pins
  for(int i=3;i<9;i++)
    pinMode(i,OUTPUT);
  for(int i=11;i<13;i++)
    pinMode(i,OUTPUT);
   
  int x = 00; // set valves to 0
  set_valve(1,x);
  set_valve(2,x);
  set_valve(3,x);
  set_valve(4,x);
  
}


void loop ()
{
  get_flow_rates();
  ctrl_pump(1, pump_pwr[0]); ctrl_pump(2, pump_pwr[1]);
  set_valve(1, valve_pos[0]); set_valve(2, valve_pos[1]);
  set_valve(3, valve_pos[2]); set_valve(4, valve_pos[3]);

  

  if (Serial.available())
  {
      int x = Serial.parseInt();
      int y = Serial.parseInt();
      int z = Serial.parseInt();

      if (x == 1)
      {
        valve_pos[0] = y;
      }
      if (x == 2)
      {
        valve_pos[1] = y;
      }       
       if (x == 3)
      {
        valve_pos[2] = y;
      }
      if (x == 4)
      {
        valve_pos[3] = y;
      }  
      pump_pwr[0] = z;
      pump_pwr[1] = z;
  }
  

  
  
}
