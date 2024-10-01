
// Hardware - Arduino Mega 2560

int u_p1 = 0;
float interval = 1;

float pump_pwr_uf[2]  = {0.0, 0.0};
float pump_pwr[2]  = {0.0, 0.0}; // Pump Power Value that gets passed into the PUMPS
float valve_pos[4] = {0.0, 0.0, 0.0, 0.0};
float flow_rate_uf[4] = {0.0, 0.0, 0.0, 0.0};
float prev_flow_rate_uf[4] = {0.0, 0.0, 0.0, 0.0};
float flow_rate[4] = {0.0, 0.0, 0.0, 0.0};
float des_flow_rate[4] = {0.0, 0.0, 0.0, 0.0};
float flow_rate_left = 0;
float flow_rate_right = 0;
int zone_id = 0;

float lookUpTableFlow_L[9] = {0, 122.0, 158.0, 192.0, 237.0, 289.0, 316.0, 352.0, 395.0}; 
//Look-up Table Values. These were collected from a experimental test
float lookUpTablePwr_L[9] = {0, 21.0, 25.0, 29.0, 34.0, 38.0, 43.0, 50.0, 56.0}; 
//Look-up Table Values. These were collected from a experimental test
float lookUpTableFlow_R[9] = {0, 122.0, 167.0, 201.0, 237.0, 281.0, 325.0, 360.0, 404.0}; 
//Look-up Table Values. These were collected from a experimental test
float lookUpTablePwr_R[9] = {0, 17.0, 22.0, 26.0, 30.0, 35.0, 40.0, 46.0, 52.0}; 
//Look-up Table Values. These were collected from a experimental test

float dfr_L = 0;
float dfr_R = 0;

float u1_base = 0; //Pump Power calculated via Lookup Table -- PUMP 1
float u1_PI = 0; //Pump Power calculated via PI Controller -- PUMP 1
float u2_base = 0; //Pump Power calculated via Lookup Table -- PUMP 2
float u2_PI = 0; //Pump Power calculated via PI Controller -- PUMP 2

int eFlowLeft = 0;
int eFlowRight = 0;
int error_i_left = 0;
int error_i_right = 0;

float int_comp = 0;
int u_pump[2] = {0, 0};

const int window_size = 200;  // define the window size.
int flow_s0_ar[window_size];
int flow_s1_ar[window_size];
int flow_s2_ar[window_size];
int flow_s3_ar[window_size];

const int window_size_pump = 70;
int pump_1_ar[window_size_pump];


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



volatile int flow_frequency18, flow_frequency19, flow_frequency20, flow_frequency21 ; 
// Measures flow sensor pulses

unsigned int l_hour18, l_hour19, l_hour20, l_hour21; // Calculated litres/hour
const byte flowsensor18  =  18;  // Flow Sensor Input 1
const byte flowsensor19  =  19;  // Flow Sensor Input 2
const byte flowsensor20  =  20;  // Flow Sensor Input 3
const byte flowsensor21  =  21;  // Flow Sensor Input 4
unsigned long currentTime;
unsigned long cloopTime;  // to measure flow rate
unsigned long cloopTime2; // to compute moving average

unsigned long currentT;
unsigned long cloopT;     // for sampling time, main loop 


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
//  Serial.print("des"); Serial.print("\t");
//  Serial.print(des_flow_rate[0], 0); Serial.print("\t"); Serial.print(des_flow_rate[1], 0); Serial.print("\t");
//  //Serial.print(des_flow_rate[2], 0); Serial.print("\t"); Serial.print(des_flow_rate[3], 0); Serial.print("\t");
//  Serial.println();

  Serial.print("flo"); Serial.print("\t");
  Serial.print(flow_rate[0], 0); Serial.print("\t"); 
  Serial.print(flow_rate[1], 0); Serial.print("\t");
  Serial.print(flow_rate[2], 0); Serial.print("\t"); 
  Serial.print(flow_rate[3], 0); Serial.print("\t");
  Serial.println();

  Serial.print("des"); Serial.print("\t");
  Serial.print(dfr_L); Serial.print("\t");
  Serial.print(dfr_R); Serial.print("\t");
  Serial.println();
  
  Serial.print("flo"); Serial.print("\t");
  Serial.print(flow_rate_left); Serial.print("\t");
  Serial.print(flow_rate_right); Serial.print("\t");
  Serial.println();
  
  Serial.print("vlv"); Serial.print("\t");
  Serial.print(valve_pos[0],0); Serial.print("\t"); 
  Serial.print(valve_pos[1],0); Serial.print("\t");
  Serial.print(valve_pos[2],0); Serial.print("\t"); 
  Serial.print(valve_pos[3],0); Serial.print("\t");
  Serial.println();

  Serial.print("pmp"); Serial.print("\t");
  Serial.print(pump_pwr[0],2); Serial.print("\t"); 
  Serial.print(pump_pwr[1], 2); Serial.print("\t");
  Serial.println();
  Serial.println();

  Serial.print("Zone: "); Serial.println(zone_id);
  
//  Serial.print("Integral Control Effort: ");Serial.print("\t");
//  Serial.println(int_comp);
//  Serial.println();
//  Serial.println();
}

void send2ros()
{
  Serial2.print(flow_rate_left,0);Serial2.print(",");
  Serial2.print(flow_rate_right,0);Serial2.print(",");
  Serial2.print(pump_pwr[0],0);Serial2.print(",");
  Serial2.print(pump_pwr[1],0);Serial2.print(",");
  Serial2.println();
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


void get_flow_rates()
{
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 23Q, Q is flow rate in L/min.

    l_hour18 = ((flow_frequency18 * 60 / 7.5) * 1.1); 
    // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency18 = 0; // Reset Counter
    flow_rate_uf[0] = l_hour18;
    
    l_hour19 = ((flow_frequency19 * 60 / 7.5) * 1.1); 
    // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency19 = 0; // Reset Counter
    flow_rate_uf[1] = l_hour19;
   
    l_hour20 = ((flow_frequency20 * 60 / 7.5) * 1.1); 
    // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency20 = 0; // Reset Counter
    flow_rate_uf[2] = l_hour20;
    
    l_hour21 = ((flow_frequency21 * 60 / 7.5) * 1.1); 
    // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency21 = 0; // Reset Counter
    flow_rate_uf[3] = l_hour21; 

     //debug_disp();
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
  Serial2.begin(57600); //Send to Ros Serial
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

int cal_pump_pwr(int r)
{
  int x = r;
  int a1 = 50;  int a2 = 200;
  int b1 = 21;  int b2 = 62;

  float f1 = float(x-50)/float(a2-a1); 
  float yy = b1 + (float(b2-b1))*f1;

  return int(yy);
}


void loop ()
{
  get_flow_rates();
  flow_rate_left = flow_rate_uf[0] + flow_rate_uf[1];
  flow_rate_right = flow_rate_uf[2] + flow_rate_uf[3];

  currentT = millis();
  if (currentT >= (cloopT + 200))
    {
      cloopT = currentT; // Updates cloopT
   

      //pump_pwr[0] = maf_pump1(pump_pwr_uf[0]);

      ctrl_pump(1, pump_pwr[0]); ctrl_pump(2, pump_pwr[1]);
      set_valve(1, valve_pos[0]); set_valve(2, valve_pos[1]);
      set_valve(3, valve_pos[2]); set_valve(4, valve_pos[3]);

      //Serial.println("testing");

      debug_disp();
      send2ros();
   
    }

  if (Serial.available())
  {
    dfr_L = Serial.parseInt();
    dfr_R = Serial.parseInt();
  }

  if (Serial2.available())
  {
    int xx = Serial2.parseInt();
    //zone_id = Serial2.parseInt();
    if (xx != 0)
    zone_id = xx;
    //Serial.println(zone_id);
  }

  for (int i = 0; i<=(sizeof(lookUpTableFlow_L)/sizeof(float))-1; i++){
    if(dfr_L>=lookUpTableFlow_L[i]){
      int indexL = i;
      int nxt_indexL = i+1;
      float weight_fxnL = (dfr_L-lookUpTableFlow_L[indexL])
      /(lookUpTableFlow_L[nxt_indexL]-lookUpTableFlow_L[indexL]);
      u1_base = lookUpTablePwr_L[indexL]+
      ((weight_fxnL)*(lookUpTablePwr_L[nxt_indexL]-lookUpTablePwr_L[indexL]));
    }
  }
for (int i = 0; i<=(sizeof(lookUpTableFlow_R)/sizeof(float))-1; i++){
    if(dfr_R>=lookUpTableFlow_R[i]){
      int indexR = i;
      int nxt_indexR = i+1;
      float weight_fxnR = (dfr_R-lookUpTableFlow_R[indexR])
      /(lookUpTableFlow_R[nxt_indexR]-lookUpTableFlow_R[indexR]);
      u2_base = lookUpTablePwr_R[indexR]+
      ((weight_fxnR)*(lookUpTablePwr_R[nxt_indexR]-lookUpTablePwr_R[indexR]));
    }
  }


  if (u1_base > 60)
    u1_base = 60;
  if (u1_base < 0)
    u1_base = 0;
  if (u1_PI > 20)
    u1_PI = 20;
  if (u1_PI < 0)
    u1_PI = 0; 
  if (u2_base > 60)
    u2_base = 60;
  if (u2_base < 0)
    u2_base = 0;
  if (u2_PI > 20)
    u2_PI = 20;
  if (u2_PI < 0)
    u2_PI = 0;   
    
    
  eFlowLeft = dfr_L - flow_rate_left;
  eFlowRight = dfr_R - flow_rate_right;

  if (abs(eFlowLeft)<10)
    error_i_left += eFlowLeft;
  if (abs(eFlowRight)<10)
    error_i_right += eFlowRight;
  
  float kpL = 0.098; //proportional gain
  float kpR = 0.102; //proportional gain
  float kiL = 0.0007; //Integral gain
  float kiR = 0.0007; //integral gain
  
  float int_left = kiL*error_i_left;
  float int_right = kiR*error_i_right;

  int z = 4;
  if (int_left > z)
    int_left = z;
  if (int_left < -z)
    int_left = -z;
  if (int_right > z)
    int_right = z;
  if (int_right < -z)
    int_right = -z;
    
  u1_PI = (kpL*eFlowLeft)+(int_left);
  u2_PI = (kpR*eFlowRight)+(int_right);

//  Serial.print("Base 1: "); Serial.println(u1_base);
//  Serial.print("Base 2: "); Serial.println(u2_base);
//  Serial.print("PI 1: "); Serial.println(int_left);
//  Serial.print("PI 2: "); Serial.println(int_right);
  
  
  pump_pwr[0] = u1_base + u1_PI;
  pump_pwr[1] = u2_base + u2_PI;
  
  //Serial.print("Error: ");Serial.println(eFlowLeft);
  //Serial.print("Base: ");Serial.println(u1_base);
  //Serial.print("PI: ");Serial.println(u1_PI);
  
  valve_pos[0] = 80;
  valve_pos[1] = 80;
  valve_pos[2] = 80;
  valve_pos[3] = 80;

}
