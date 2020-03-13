#include "modularfeedback.h"

#define ctrl_pin 4 //define the pin that is used for control
#define sensor_pin 2 //define the analog pin on which the senor is connected
#define gain_system1 1.0  //define system1 gain as 1 - default value to use for calibration
#define offset_system1 0.0 //define system1 offset as 0 - default value to use for calibration

//To run these tests, uncomment "debug = true" in modularfeedback.cpp (in the UpdateFeedback function)
//REMEMBER TO COMMENT BACK WHEN YOU ARE DONE because otherwise the code won't work properly 

//to do: add more tests 

void testcase1(){ 
  Serial.println("TEST CASE 1: DIRECT = 1, RELAY INITIALLY ENABLED ");
  NegFeedback system1(sensor_pin, ctrl_pin, 400, 1.5, 2.0, 1, 5000, gain_system1, offset_system1, 0);
  
  bool rlystatus2 = system1.UpdateFeedback(true); //RELAY DISABLED, RELAY STATUS AFTER UPDATE SHOULD BE 0
  Serial.print("EXPECTED VALUE = 0, ACTUAL VALUE IS: ");
  Serial.println(rlystatus2);

  bool rlystatus1 = system1.UpdateFeedback(false); //RELAY ENABLED, RELAY STATUS AFTER UPDATE SHOULD BE 1
  Serial.print("EXPECTED VALUE = 1, ACTUAL VALUE IS: ");
  Serial.println(rlystatus1);

  Serial.println("FINISHED");
  
}



 
void setup() {
  Serial.begin(9600);
  testcase1();


}

void loop() {
}
