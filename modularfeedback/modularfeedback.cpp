//Enoch Chau 2020: Adopted from Dr. Michael Klopfer, built for modularity
//Melinda Tran, Tritai Nguyen Winter 2020: Modified library written by Enoch Chau
//Negative feedback
#include "modularfeedback.h"
//Output options:
#define DEBUG  //This line enables serial printout for debugging purpouses in the mechanics of the Negative feedback function, please comment it out to turn off serial printouts

NegFeedback::NegFeedback(int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, bool disable){
  //initialize local variables with the initial object call
  pressure_set1 = setp;
  Sensor_Pin = readpin;
  RelayCtrl_1_Pin = ctrlpin;
  direct = dir;
  lower_hysteresis_1 = lowhys;
  upper_hysteresis_1 = highhys;
  min_trans_time = trans;
  PC1_gain = gain;
  PC1_offset = offset;

  global_disable = disable; //global variable that acts as a kill switch for the relay (enables relay when false, disables relay when true)

  //Initiate heating element relay
  pinMode(RelayCtrl_1_Pin, OUTPUT);  //Initiate system element relay, set as output (solenoids, pumps, etc.)
  if(global_disable){
    digitalWrite(RelayCtrl_1_Pin, LOW); //Initiate system element relay, start LOW (off), this is typical
    if(direct)
      RelayStatus_1 = 0; 
    else
      RelayStatus_1 = 1;
  }
  else{
    digitalWrite(RelayCtrl_1_Pin, HIGH); //Initiate system element relay, start HIGH (on) - this is atypical, you should start with this off
    if(direct)
      RelayStatus_1 = 1;
    else
      RelayStatus_1 = 0;
  }

}

//Read sensor and return data
double NegFeedback::ReadSensor (int Sensor_Pin) {
  double data = analogRead(Sensor_Pin);
  return data;
}

void NegFeedback::ChangeSetPoint (double setp){ //change of setpoint in active usage, call before the update function, and next time it is called, it will be updated.
  pressure_set1 = setp;
}

void NegFeedback::ReInitializeSystem (int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, bool disable){ //Used to change all the object parameters, note the "initialstate" should typically be -1 if this is called to use the last state to reduce chance of a discontinuity problem
  //Caution, this may cause a momentary discontinuity in action, call only when the system is stable and this can happen without consequence.  This is provided as a conventience but should not be something changed regularly
  //Reinitialize local variables
  Sensor_Pin = readpin;
  RelayCtrl_1_Pin = ctrlpin;
  direct = dir;
  lower_hysteresis_1 = lowhys;
  upper_hysteresis_1 = highhys;
  min_trans_time = trans;
  PC1_gain = gain;
  PC1_offset = offset;

 //Re-Initiate heating element relay 
  if (disable == 1)  //relay disabled 
  {
    digitalWrite(RelayCtrl_1_Pin, LOW); //Initiate system element relay, start LOW (off) 
    global_disable = disable;
  
  }
  else if (disable == 0) //relay enabled 
  {
    digitalWrite(RelayCtrl_1_Pin, HIGH); //Initiate system element relay, start HIGH (on) - this is atypical, you should start with this off
    global_disable = disable;
  }
  else
  {
    //Just ignore the change and leave it at the last state, this is the default option (you can use anything but 0 or 1, so just use -1 as the arguement value to get here).
  }
}

bool NegFeedback::UpdateFeedback(bool disable){ //called to check value and update status against set point.  Call continuously in usage.
  //Initialization of common control variables
  bool debug = false;
  const int sampleloop = 10; //number of averaged reads for calculation (increased number reduces outliers and improves percision at the cost of responsiveness) (Needs to be const to allow this variable to size an array)

  double rawpressure_read1 = 0;
  double pressure_read1 = 0;  //Pressure that system element is currently maintaining - initialize the pressure variable for the averages
  double samples[sampleloop]={0};
  for (int i=0; i<sampleloop; i++)//read sequential samples
    {
      samples[i] = ((double)PC1_gain*(analogRead(Sensor_Pin)))+PC1_offset;  //Measurement and calibration of PC input
    }
  for (int i=0; i<sampleloop; i++) //average the sequential samples
    {
      rawpressure_read1=samples[i]+rawpressure_read1;
    }
  rawpressure_read1=rawpressure_read1/(double)sampleloop;
  pressure_read1 = rawpressure_read1;

  #ifdef DEBUG
  debug = true; //uncomment this to test the disable function; added this because i couldn't get the right readings and values to trigger the if statements 
  Serial.print("Raw Sensor average ADC values:"); Serial.print(rawpressure_read1); Serial.print(" ");
  Serial.print("Calibrated Sensor value (your calibrated units)):"); Serial.print(pressure_read1); Serial.print(" "); //Display averaged PC temperature
  #endif
   runtime=millis(); //set runtime

  if (lastruntime>runtime) //check for millis() rollover event, prepare accordingly, skip and wait until time reaccumulates
    {
      overrideindicator=1;
      lastruntime=runtime;
      delay (min_trans_time); //delay if overflow event is detected as failsafe
    }

  if (direct==true){
    if ((RelayStatus_1==0 && pressure_read1<=pressure_set1-lower_hysteresis_1  && min_trans_time<(lastruntime-runtime) && !overrideindicator) || debug){

             //(disable==true) ? digitalWrite(RelayCtrl_1_Pin, LOW) : digitalWrite(RelayCtrl_1_Pin, HIGH); //turns relay off if true, turns relay on otherwise
             if(disable){
               digitalWrite(RelayCtrl_1_Pin,LOW);
               //RelayStatus_1 = 0;
             }

             else{
               digitalWrite(RelayCtrl_1_Pin,HIGH);
               //RelayStatus_1 = 1;
             }
             lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
             RelayStatus_1=1;  //toggle relay status indicator
             lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
             overrideindicator=0; //reset millis() overflow event indicator
          }

    else if (RelayStatus_1==1 && pressure_read1>=pressure_set1+upper_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator){
              digitalWrite(RelayCtrl_1_Pin, LOW);
              lastswitcheventtime =  runtime-lastswitcheventtime; //reset transition time counter (verify no issue with millis() rollover)
              RelayStatus_1=0;  //toggle relay status indicator
              lastruntime=runtime;   //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
              overrideindicator=0; //reset millis() overflow event indicator
          }

  else {
    
    //Ignore change and leave it in last state
  }
  }

if (direct==!true)
  {
    if ((RelayStatus_1==1 && pressure_read1<=pressure_set1-lower_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator) || debug)
     {
        if(disable){
          digitalWrite(RelayCtrl_1_Pin,LOW);
	        //RelayStatus_1 = 1;
        }

        else{
        digitalWrite(RelayCtrl_1_Pin,HIGH);
        //RelayStatus_1 = 0;
        } 

      //(disable==true) ? digitalWrite(RelayCtrl_1_Pin, LOW) : digitalWrite(RelayCtrl_1_Pin, HIGH); //turns relay off if true, turns relay on otherwise 
      lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
      RelayStatus_1=0;  //toggle relay status indicator
      lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
      overrideindicator=0; //reset millis() overflow event indicator
       }
    else if (RelayStatus_1==0 &&/* pressure_read1>=pressure_set1+upper_hysteresis_1 && */min_trans_time<(lastruntime-runtime) && !overrideindicator)
        {
         digitalWrite(RelayCtrl_1_Pin, LOW);
         lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
         RelayStatus_1=1;  //toggle relay status indicator
         lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
       }
  else {
    //Ignore change and leave it in last state
  }
  }

//Serial DEBUG printout when DEBUG line is active
  #ifdef DEBUG
  Serial.print("SetPoint (your calibrated units):"); Serial.print(pressure_set1); Serial.print(" ");
  Serial.print("Avg. Calibrated Sensor Value(your calibrated units)):"); Serial.print(pressure_read1); Serial.print(" "); //Display averaged PC temperature
  Serial.print("Current Control Relay Status:"); Serial.print(RelayStatus_1); Serial.print(" "); //Display the present status of the element control relay
  Serial.print("Time between the last switch to the Present Switch State:"); Serial.print(lastswitcheventtime); Serial.print(" "); Serial.print("TotalRuntime(ms):"); Serial.println(runtime);
  #endif
  global_disable = disable;
  return RelayStatus_1; //return the relay status value to function call; if global_disable = true, returns what relay status would have been if relays were enabled 
  }
