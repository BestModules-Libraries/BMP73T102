/*****************************************************************
File:        rotatingStepperMotor.ino 
Description: Initialize the maximum speed and initial speed after power on,
             the stepper motor circulates the following steps: 
             1. run 800 steps clockwise at the speed of 800,wait for 1 second;
             2. run to the 0 position with an acceleration of 400,wait for 1 second;            
******************************************************************/
#include <BMP73T102.h>
BMP73T102 MyStepper(2);//2:Half step drive 

void setup() {
    MyStepper.begin();
    MyStepper.setStepperMaxSpeed(1000);
    MyStepper.setStepperAcceleration(400);
}

void loop() { 
    MyStepper.stepperMove(800, 800);  
    delay(1000);
    MyStepper.stepperMoveTo(0); 
    delay(1000);  
}
