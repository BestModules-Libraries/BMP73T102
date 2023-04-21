/*****************************************************************
File:             BMP73T102.h
Author:           Liu, BESTMODULES
Description:      Define classes and required variables
History:         
V1.0.1  -- initial version;2023-01-09;Arduino IDE : v1.8.15
******************************************************************/
#ifndef BMP73T102_h
#define BMP73T102_h

#include <Arduino.h>

#define DC1        (1)      //DC motor1
#define DC2        (2)      //DC motor2    
#define FORWARD    (0)      //Indicates clockwise rotation 
#define REVERSAL   (1)      //Indicates counterclockwise rotation 
#define FULL4WIRE  (1)      //Fall step drive 
#define HALF4WIRE  (2)      //Half step drive  

class BMP73T102
{ 
    public:
        BMP73T102(); 
        BMP73T102(uint8_t interface, uint8_t pin1 = 9, uint8_t pin2 = 6, uint8_t pin3 = 5, uint8_t pin4 = 3);
        void begin();
        /*dc motor*/              
        void dcMotorRun(uint8_t ch, int8_t rank); 
        void dcMotorStop(uint8_t ch);
        void dcMotorBrake(uint8_t ch); 
        int8_t getDcMotorRank(int8_t ch);                                
        /*stepper motor*/       
        void stepperMoveTo(int32_t absolute, uint16_t speed);    
        void stepperMoveTo(int32_t absolute);
        void stepperMove(int32_t relative, uint16_t speed);
        void stepperMove(int32_t relative);   
	uint16_t getStepperMaxSpeed();
        uint16_t getStepperAcceleration(); 
        int32_t getStepperPosition();     
        void setStepperMaxSpeed(uint16_t maxSpeed); 
        void setStepperAcceleration(uint16_t acceleration);        
        void setStepperCurrentPosition(int32_t position);
        
           
    private:
        /*dc motor*/      
        int8_t _rank1;                 //DC motor1 speed gear, range -100~100
        int8_t _rank2;                 //DC motor2 speed gear, range -100~100        
     
        /*stepper motor*/
        void setSpeed(uint16_t speed);//
        void computeNewSpeed();        
        void step(long step);
        boolean run();
        void stepperKeepRun();
        int32_t getStepperDistanceToGo();
        
        uint8_t _interface;            //Driving mode,FULL4WIRE=0 Fall step drive or HALF4WIRE=1 Half step drive 
        long  _currentPos;             //The current position in steps    
        long  _targetPos;              //The target position in steps
        float _speed;                  //The current motos speed in steps per second
        float _maxSpeed;               //The maximum permitted speed in steps per second. Must be > 0.
        unsigned long  _stepInterval;  //The current interval between steps in microseconds
        unsigned long  _lastStepTime;  //The last step time in microseconds
        uint8_t _dir;                  //Direction of stepper motor 
        long _distanceTo;              //The distance from the current position to the target position.          
        long _count;                   //The step counter for speed calculations 
        float _initStepInterval;       //Initial step size in microseconds   
        float _nextStepInterval;       //Last step size in microseconds  
        float _minStepInterval;        //Min step size in microseconds based on maxSpeed
        float _acceleration;           //The acceleration to use to accelerate or decelerate the motor in steps,Must be > 0
        uint8_t _flagAccelerate;       //The flag of acceleration run
        uint8_t _flagHoldTorque;       //The flag of maintain the torque
        uint8_t _flagStop;             //The flag of stop working   
        long _everyInitPosition;       //Every start position
        /*Four pins of stepper motor;_pin1 and _pin2 are stepper one phase,_pin3 and _pin4 are stepper another phase*/
        uint8_t _pin1;                  
        uint8_t _pin2;
        uint8_t _pin3;
        uint8_t _pin4; 
          
}; 
#endif
