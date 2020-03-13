//Example Negative Feedback Control: Object orriented loop for Arduino using an analog read from a pressure sensor.  This code serves as a template/example for further development and integraton into specific applications
//University of California, Irvine
//California Institute of Telecommunications and Information Technology
//Michael J. Klopfer, Ph. D. - 2017
//updated 3/3/2020
//Note: In future updates we may want to include a disable option in the UPDATE function such that no matter what the current read value is, the control relay is disabled based on a variable input (e.g. 1=disabled, 0=normal operation)  The function will return the state without providing control, knowledge of the disabled bit setting and this requested state will indicate if the relay is actually being driven.  THis can be used as an ESTOP functional control or pause. 

//Output options:
#define DEBUG  //This line enables serial printout for debugging purpouses in the mechanics of the Negative feedback function, please comment it out to turn off serial printouts

//Configuration values:
#define ctrl_pin 4 //define the pin that is used for control
#define sensor_pin 0 //define the analog pin on which the senor is connected
#define gain_system1 1.0  //define system1 gain as 1 - default value to use for calibration
#define offset_system1 0.0 //define system1 offset as 0 - default value to use for calibration

//******************Class for negative feedback control*******************
class NegFeedback
{
  // Class Member Variables
  // These are initialized at startup
  //Interface Configuration Pins
  //NOTE, these variables are not protected, but please use the update functions rather than direct modification of class members, update this class classy, not trashy
  
  //Sensor calibration factors - this lets you use a sensor with a 
  float PC1_gain; //Pressure sensor correction gain
  float PC1_offset;  //Pressure sensor correction offset

  //Hardware Pins - define your sensor pins and the output control pin for your connected load (assumed via a relay)
  int Sensor_Pin;
  int RelayCtrl_1_Pin;

  int direct; //direct or inverse control (direct (true) = heating, reverse (false) = cooling) - basically does the action of your controlled device move the value toward your setpoint in a positive or negative direction.  Get this wrong and you will not get control, your controlled device wills tay on forever and keep moving in the wrong direction from the set point.
  double upper_hysteresis_1; //value for overshoot hysteresis (your calibrated units)
  double lower_hysteresis_1; //value for undershoot hysteresis (your calibrated units)
  int min_trans_time; // minimum time in a state (value in ms), prevents rapid switching for compressors or likewise which can not be rapidly cycled
  double pressure_set1;  //  Control setpoint value (your calibrated units)
  unsigned long runtime=0; //tracker for runtime
  bool RelayStatus_1=0; //status of relay
  bool overrideindicator=0; //used to detect millis() rollover

  bool disable; //global var to enable and disable relays

  // These maintain the current state
  unsigned long lastruntime=0;//used to detect millis() rollover and indicate transitions
  unsigned long lastswitcheventtime=0; //time in millis() since last switch event


  // Constructor - creates a NegFeedback Object 
  // and initializes the member variables and state
 public:
  NegFeedback(int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, bool initialstate)
  {
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

  disable = false; //enables relay

    //Initiate heating element relay
  pinMode(RelayCtrl_1_Pin, OUTPUT);  //Initiate system element relay, set as output (solenoids, pumps, etc.)
  if (initialstate == 1)  //Initialstate=1
    {
      digitalWrite(RelayCtrl_1_Pin, HIGH); //Initiate system element relay, start HIGH (on) - this is atypical, you should start with this off
    }
    else
    {
      digitalWrite(RelayCtrl_1_Pin, LOW); //Initiate system element relay, start LOW (off), this is typical
    }
  }
  
  void ChangeSetPoint (double setp) //change of setpoint in active usage, call before the update function, and next time it is called, it will be updated.
  {
    pressure_set1 = setp;
  }
    
  void ReInitializeSystem (int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, int initialstate) //Used to change all the object parameters, note the "initialstate" should typically be -1 if this is called to use the last state to reduce chance of a discontinuity problem
  {
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
    if (initialstate == 1)  //Initialstate=1
    {
      digitalWrite(RelayCtrl_1_Pin, HIGH); //Initiate system element relay, start HIGH (on) - this is atypical, you should start with this off
    }
    else if (initialstate == 0)
    {
      digitalWrite(RelayCtrl_1_Pin, LOW); //Initiate system element relay, start LOW (off)
    }
    else
    {
      //Just ignore the change and leave it at the last state, this is the default option (you can use anything but 0 or 1, so just use -1 as the arguement value to get here).
    }  
  }
    
  bool UpdateFeedback(bool disable) //called to check value and update status against set point.  Call continuously in usage.
    {
    //Initialization of common control variables
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
    //Serial.print("Raw Sensor average ADC values:"); Serial.print(rawpressure_read1); Serial.print(" ");
    //Serial.print("Calibrated Sensor value (your calibrated units)):"); Serial.print(pressure_read1); Serial.print(" "); //Display averaged PC temperature
    #endif
     runtime=millis(); //set runtime
   
    if (lastruntime>runtime) //check for millis() rollover event, prepare accordingly, skip and wait until time reaccumulates
      {
        overrideindicator=1;
        lastruntime=runtime;
        delay (min_trans_time); //delay if overflow event is detected as failsafe
      }
  
    if (direct==true)
       {
           if (RelayStatus_1==0 && pressure_read1<=pressure_set1-lower_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
           {  
             (disable==true) ? digitalWrite(RelayCtrl_1_Pin, LOW) : digitalWrite(RelayCtrl_1_Pin, HIGH); //turns relay off if true, turns relay on otherwise
               //digitalWrite(RelayCtrl_1_Pin, HIGH);
               lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
               RelayStatus_1=1;  //toggle relay status indicator
               lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
               overrideindicator=0; //reset millis() overflow event indicator
            }
     
     else if (RelayStatus_1==1 && pressure_read1>=pressure_set1+upper_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
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
      if (RelayStatus_1==1 && pressure_read1<=pressure_set1-lower_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
       {
        (disable==true) ? digitalWrite(RelayCtrl_1_Pin, LOW) : digitalWrite(RelayCtrl_1_Pin, HIGH);
        //digitalWrite(RelayCtrl_1_Pin, HIGH);
        lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
        RelayStatus_1=0;  //toggle relay status indicator
        lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
        overrideindicator=0; //reset millis() overflow event indicator
            
         }
      else if (RelayStatus_1==0 && pressure_read1>=pressure_set1+upper_hysteresis_1 && min_trans_time<(lastruntime-runtime) && !overrideindicator)
          {
           digitalWrite(RelayCtrl_1_Pin, LOW);
           lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
           RelayStatus_1=1;  //toggle relay status indicator
           lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
           overrideindicator=0; //reset millis() overflow event indicator
         } 
    else {}
    }
  
  //Serial DEBUG printout when DEBUG line is active
    //#ifdef DEBUG
    Serial.print("SetPoint (your calibrated units):"); Serial.print(pressure_set1); Serial.print(" ");
    Serial.print("Avg. Calibrated Sensor Value(your calibrated units)):"); Serial.print(pressure_read1); Serial.print(" "); //Display averaged PC temperature
    Serial.print("Current Control Relay Status:"); Serial.print(RelayStatus_1); Serial.print(" "); //Display the present status of the element control relay
    Serial.print("Time between the last switch to the Present Switch State:"); Serial.print(lastswitcheventtime); Serial.print(" "); Serial.print("TotalRuntime(ms):"); Serial.println(runtime);
    //#endif
    disable = false;
    return RelayStatus_1; //return the relay status value to function call
    }


};

//*********************************************************************

//User Code

//Call a single (you can have multiple) instance of the negative feedback control object
//In this case we are showing an example of a pressure senor where when the pressure rises above a certain threshold a feed solenoid is turned off (positive direction negative feedback control)
//The raw ADC values have been calibrated to units of PSI using a manometer, such that using Excel, a linear regression was found such that the gain (m) and offset (b) of the regression (where the reporting variable (x) is ADC value and the responding variable (y) is the calibrated value in PSI
//A value of 70 PSI (based on the calibrated value) was used as the set point for control
//If you want to work in raw ADC values (unless you are a masochist, this should be temporary), you can always set gain=1 and offset=0 so that y=x and use a setpoint in ADC values
//

//Decalare Negative Feedback object instance
NegFeedback system1(sensor_pin, ctrl_pin, 40, 1.5, 2.0, 1, 5000, gain_system1, offset_system1, 0); 
//Define "system1" object with parameters - 
//Template:  NegFeedback(int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, bool initialstate)

void setup()
{
  Serial.begin(115200);  //Move serial fast to improve 
  pinMode(ctrl_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);

  //Define output pins
}

void loop()
{  
  bool rlystatus = system1.UpdateFeedback(); //run to sample and return relay status - more variables can be returned using a structure, holder for the status of the actuated relay
  Serial.print("Present Control Relay Status:"); Serial.println(rlystatus); //Display the present status of the element control relay returned to the main function for this object call
  
  //If you want to change the setpoint call the following function [shown for an example of 50 PSI (calibrated units in this example)]:
  //system1.ChangeSetPoint (50) 
  
  //If you really want to change the parameters of your negative feedback function (not typical!) use the following function (shown as initializing with the same initial values - pointless case example)
  //system1.ReInitializeSystem (0, 4, 70, 1.5, 2.0, 1, 5000, 0.7789, -17.596, -1)  // Template: NegFeedback(int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, int initialstate) //Note, for the last arguement, -1 is used to keep the relay in the present state rather than forcing it either off or on. 

  delay (100); //delay before rechecking and updating read and control status
}

