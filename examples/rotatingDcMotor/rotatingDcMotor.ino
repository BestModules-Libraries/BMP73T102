/*****************************************************************
File:        rotatingDcMotor.ino   
Description: After power on, the DC motor circulates the following steps: 
             1. DC motor 1 rotates counterclockwise for 3S at the speed gear of 50, and then stops for 1s;
             2. DC motor 2 rotates clockwise for 3S at the speed gear of 80, and then stops for 1s. 
******************************************************************/
#include <BMP73T102.h>
BMP73T102 dcmotor;

void setup() {
    dcmotor.begin();
}

void loop() { 
    dcmotor.dcMotorRun(1,-50); 
    dcmotor.dcMotorRun(2,80); 
    delay(3000);
    dcmotor.dcMotorStop(1);
    dcmotor.dcMotorStop(2);
    delay(1000);

}
