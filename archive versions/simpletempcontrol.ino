//Simple Arduino Feedback Temperature controller with hysteresis for Relay and Thermocouple measurement - Example
//Michael Klopfer, PhD  California Plug Load Research Center, University of California, Irvine
//Built to work with K-Type thermocouple and MAX31855 demo board from Adafruit Industries (Adafruit library by Limour Fried)
//Version 1.0 (Feb 26, 2017)
#include <SPI.h>
#include "Adafruit_MAX31855.h"
 
//
//Interface Configuration Pins
//
//MAX31855 thermocouple interface. (By using separate CSBs, multiple devices can use the same SPI bus - implemented only for 1 device in this example)
#define maxCS 6 //CS of a MAX31855 thermocouple interface. By using separate CSBs, multiple devices can use the same SPI bus.
#define maxSO  5 //SPI MOSI pin of MAX31855 thermocouple interface
#define maxSCK  7 //SPI CLK pin of MAX31855 thermocouple interface
//Output Relay
#define RelayCtrl_1_Pin  4 //Pin to control relay that switches the thermal control
#define direct true //direct or inverse control (direct (true) = heating, reverse (false) = cooling)
#define upper_hysteresis_1 1.5  //value for overshoot hysteresis (degrees)
#define lower_hysteresis_1 2  //value for undershoot hysteresis (degrees)
#define min_trans_time 5000 // minimum time in a state (value in ms), prevents rapid switching for compressors or likewise which can not be rapidly cycled
int sampleloop = 10; //number of averaged reads for calculation (increased number reduces outliers and improves percision at the cost of responsiveness)
double temp_set1 = 125.0;  // *F  - Control setpoint value (in degrees F)  (degrees F is used because of marginally higher resolution per degree)
//Initialization of control variables
unsigned long lastswitcheventtime=0; //time in millis() since last switch event
double temp_read1;  //Temperature that system is currently maintaining
unsigned long runtime=0; //tracker for runtime
unsigned long lastruntime=0;//used to detect millis() rollover and indicate transitions
byte TCError1=false; //this is a flag if there is a Thermocouple error
byte RelayStatus_1=0; //status of relay
byte overrideindicator=0; //used to detect millis() rollover
Adafruit_MAX31855 kTC(maxSCK, maxCS, maxSO); //Create a MAX31855 object with program parameters
//Thermocouple calibration constants
#define TC1_gain 0.988 //Thermocouple correction gain
#define TC1_offset -2.5  //Thermocouple correction offset
void setup()
{
  //Initiate heating element relay
  pinMode(RelayCtrl_1_Pin, OUTPUT);  //Initiate heating element relay, set as output
  digitalWrite(RelayCtrl_1_Pin, LOW); //Initiate heating element relay, start LOW (off)
  SPI.begin();   //Initiate thermocouple communication
  delay(500);   // The MAX31855 needs a little time to stabilize
  Serial.begin(115200);  //Move serial fast to improve responsiveness
}
void loop()
{
  temp_read1=0; //initialize the temp variable for the averages
  double samples[sampleloop]={0};
  for (int i=0; i<sampleloop; i++)//read sequential samples
  {
   samples[i] = (TC1_gain*kTC.readFarenheit())+TC1_offset;  //Measurement and calibration of TC input 
  }
  for (int i=0; i<sampleloop; i++) //average the sequential samples
  {
  temp_read1=samples[i]+temp_read1;
  }
  temp_read1=temp_read1/(double)sampleloop;
  
  if(isnan(temp_read1)) //check for NAN, if this is not done, if the TC messes up, the controller can stick on!
  {
    temp_read1=temp_set1*100; //fail safe! False value but way above what is expected - this approach may not always be safe for refrigeration case!!    TCError1=true;
  }
  else
  {
    TCError1=false;
  }
 runtime=millis(); //set runtime
 if (lastruntime>runtime) //check for millis() rollover event, prepare accordingly, skip and wait until time reaccumulates
 {
  overrideindicator=1;
  lastruntime=runtime;
  delay (min_trans_time); //delay if overflow event is detected as failsafe
 }
 
if (direct==true)
{
  if (RelayStatus_1==0 && temp_read1<=temp_set1-lower_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
  {
          digitalWrite(RelayCtrl_1_Pin, HIGH);
          lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
          RelayStatus_1=1;  //toggle relay status indicator
          lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
          overrideindicator=0; //reset millis() overflow event indicator
          
  }
   else if (RelayStatus_1==1 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
    {
         digitalWrite(RelayCtrl_1_Pin, LOW);
         lastswitcheventtime =  runtime-lastswitcheventtime; //reset transition time counter (verify no issue with millis() rollover)
         RelayStatus_1=0;  //toggle relay status indicator
         lastruntime=runtime;   //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
    } 
  else {}
}
if (direct==!true)
{
  if (RelayStatus_1==1 && temp_read1<=temp_set1-lower_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
  {
      digitalWrite(RelayCtrl_1_Pin, LOW);
      lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
      RelayStatus_1=0;  //toggle relay status indicator
      lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
      overrideindicator=0; //reset millis() overflow event indicator
          
  }
   else if (RelayStatus_1==0 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
    {
         digitalWrite(RelayCtrl_1_Pin, HIGH);
         lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
         RelayStatus_1=1;  //toggle relay status indicator
         lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
    } 
  else {}
}
  Serial.print("SetPoint(F):"); Serial.print(temp_set1); Serial.print(" ");
  Serial.print("AmbientCJTemp(C):"); //MAX31855 Internal Cold junction temp reading (in C) (roughly ambient temp to the IC)
  Serial.print(kTC.readInternal()); Serial.print(" "); //Read temp from TC controller internal cold junction  
  Serial.print("AvgCurrentTCTemp(F):"); Serial.print(temp_read1); Serial.print(" "); //Display averaged TC temperature
  Serial.print("RelayStatus:"); Serial.print(RelayStatus_1); Serial.print(" "); //Dis[play the present status of the thermal control relay
  Serial.print("TimeFromLastToPresentSwitchState:"); Serial.print(lastswitcheventtime); Serial.print(" "); Serial.print("TotalRuntime(ms):"); Serial.println(runtime);
  
}
