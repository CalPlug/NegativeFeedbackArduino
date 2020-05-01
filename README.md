# NegativeFeedbackArduino
***Development by Melinda Tran, Tritai Nguyen, and Enoch Chau***

The NegFeedback class allows for thermostat style control with hysteresis in an object oriented style intended for the Arduino. 
 
NegFeedbackSensor.ino is an example utilizing this class to control a relay using readings from a pressure sensor. When the pressure rises above a certain threshold, a solenoid is turned off (positive direction negative feedback control). 
 
Although NegFeedback is directly written into this file, this class has been written into the library modularfeedback for outside use. 

# How to Use
In your sketch, be sure to include the modularfeedback library. Create a NegFeedback object using the constructor.
**Note:** You will need calibrated values for overshoot hysteresis, undershoot hysteresis, and setpoint value. Raw values can be used by setting gain=1 and offset=0. 

NegFeedback(int readpin, int ctrlpin, double setp, double highhys, double lowhys, int dir, int trans, float gain, float offset, bool disable);

The parameters are as follows:

**readpin:** pin for sensor input, generally some analog input

**ctrlpin:** pin for control, usually a relay

**setp:** control setpoint value (calibrated units)

**highhys:** value for overshoot hysteresis (calibrated units)

**lowhys:** value for undershoot hysteresis (calibrated units)

**dir:** direct or inverse control; basically, does the action of the controlled device move to the setpoint in a positive (direct) direction or inverse (negative) direction 

**trans:** minimum time in a state (in ms); meant to prevent rapid switching

**gain:** sensor correction gain

**offset:** sensor correction offset

**disable:** “kill switch” for whatever is at the ctrl pin. When this value is true, it disables that device, and when false, enables it. Regardless of whether or not the control is enabled, the UpdateFeedback function will return whatever state it would be in as if it were enabled. 

In setup() function, define the data rate in bits per second (baud) for serial data transmission. Also define the input and output pins for sensing and control.
```
void setup()  {
	Serial.begin(115200);
	pinMode(ctrl_pin, OUTPUT);
	pinMode(sensor_pin, INPUT);
}
```

In loop() function, use UpdateFeedback to sample and return relay status. In the example below, system1 is the name of the instance of the negative feedback control object.

```
bool rlystatus = system1.UpdateFeedback();
```

In order to change parameters, use ChangeSetPoint and ReInitializeSystem functions defined below. One may decide to include a delay in order to define intervals between sampling data.

#Functions
```
void ChangeSetPoint(double setp);
```
Changes the setpoint during active usage; call before calling UpdateFeedback so that next time it is called, feedback will be relative to new setpoint value. 

`void ReInitializeSystem(int readpin,int ctrlpin, double setp, double highhys,double lowhys, int dir, int trans, float gain, float offset, bool disable, int initialstate);`
Used to change all object parameters during active usage. initialstate is the status of the control pin. 1= high, 0 = low, and to use previous state, set initialstate=-1. Caution: may cause momentary discontinuity in action. Only call when the system is stable. 

`bool UpdateFeedback(bool disable);`
Checks values and updates status against setpoint. To be called regularly during usage. 
Device on the control pin can be disabled during usage. Regardless of status of the control, UpdateFeedback will always return whatever state it would be in as if it were enabled. 
