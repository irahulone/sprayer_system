
const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

int x = 0; int y = 0; int z = 0;


volatile int flow_frequency2, flow_frequency3, flow_frequency18, flow_frequency19 ; // Measures flow sensor pulses
unsigned int l_hour2, l_hour3, l_hour18, l_hour19; // Calculated litres/hour
const byte flowsensor2  =  2;  // Sensor Input 1
const byte flowsensor3  =  3;  // Sensor Input 2
const byte flowsensor18 = 18;  // Sensor Input 3
const byte flowsensor19 = 19;  // Sensor Input 4
unsigned long currentTime;
unsigned long cloopTime;
void flow2 () // Interrupt function
{
  flow_frequency2++;
}
void flow3 () // Interrupt function
{
  flow_frequency3++;
}
void flow18 () // Interrupt function
{
  flow_frequency18++;
}
void flow19 () // Interrupt function
{
  flow_frequency19++;
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
  int max_pwr = 600;
  if(val < 0) val = 0;
  else if(val > max_pwr) val = max_pwr;

  if (ch==1)
  {
    Serial1.print("!G 1 ");
    Serial1.print(String(val));
    Serial1.print("_\r");
  }

  if (ch==2)
  {
    Serial1.print("!G 2 ");
    Serial1.print(String(val));
    Serial1.print("_\r");
  }
}

void setup()
{
  pinMode(flowsensor2,  INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor3,  INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor18, INPUT_PULLUP);  // Optional Internal Pull-Up
  pinMode(flowsensor19, INPUT_PULLUP);  // Optional Internal Pull-Up

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(flowsensor2),  flow2,  RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor3),  flow3,  RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor18), flow18, RISING); // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(flowsensor19), flow19, RISING); // Setup Interrupt
  currentTime = millis();
  cloopTime = currentTime;

  Serial.begin(9600);
  Serial1.begin(115260);  // roboteq controller

  for(int i=3;i<9;i++)
    pinMode(i,OUTPUT);
  for(int i=11;i<13;i++)
    pinMode(i,OUTPUT);
  
  set_valve(1,0);
  set_valve(2,0);
  set_valve(3,0);
  set_valve(4,0);
  
}

void loop ()
{

  if (Serial.available())
  {
      x = Serial.parseInt();
      y = Serial.parseInt();
      z = Serial.parseInt();
      set_valve(x, y);

  }

  ctrl_pump(1, z);

  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 23Q, Q is flow rate in L/min.
   
    l_hour2 = ((flow_frequency2 * 60 / 23) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency2 = 0; // Reset Counter
    //Serial.print(l_hour2, DEC); // Print litres/hour
    //Serial.println(" L/hour 2 ");

    l_hour3 = ((flow_frequency3 * 60 / 23) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency3 = 0; // Reset Counter
    //Serial.print(l_hour3, DEC); // Print litres/hour
    //Serial.println(" L/hour 3 ");

    l_hour18 = ((flow_frequency18 * 60 / 23) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency18 = 0; // Reset Counter
    //Serial.print(l_hour18, DEC); // Print litres/hour
    //Serial.println(" L/hour 18 ");

    l_hour19 = ((flow_frequency19 * 60 / 23) * 1.1); // (Pulse frequency x 60 min) / 23Q = flowrate in L/hour
    flow_frequency19 = 0; // Reset Counter
    Serial.print(l_hour19, DEC); // Print litres/hour
    Serial.println(" L/hour 19 ");
  }
}
