/*****************************************************************
File:          BMP73T102.cpp
Author:        Liu, BESTMODULES
Description:   Stepper motor library and DC motor library for Arduino 
History:         
V1.0.1  -- initial version;2023-01-09;Arduino IDE : v1.8.15
**************************************************************************/
#include "BMP73T102.h"
/**********************************************************
Description: DC motor constructor 
Parameters:                                                
Return:             
Others: 9 and 6 control DC motor M1, 5 and 3 control DC motor M2    
***************************************************************************/
BMP73T102::BMP73T102()
{
    _pin1 = 9;
    _pin2 = 6;
    _pin3 = 5;
    _pin4 = 3;
 }
 /****************************************************************************************
Description: Stepper motor constructor 
Parameters:  interface: driving mode:
                                     FULL4WIRE/1: fall step drive,(200 steps/circle) 
                                     HALF4WIRE/2: half step drive,(400 steps/circle)
             pin1, pin2, pin3 and pin4: Stepper motor connect ports,and respectively connected to ports 9, 6, 5 and 3 
Return:             
Others:      The wiring port can be changed as required
*****************************************************************************************/
BMP73T102::BMP73T102(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{ 
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
    _interface = interface;
    _lastStepTime = 0;
    _stepInterval = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0;
    _maxSpeed = 1;
    _acceleration = 0;
    _count = 0;
    _initStepInterval = 0;
    _nextStepInterval = 0;
    _minStepInterval = 1;
    _flagAccelerate = 0;
    _flagHoldTorque = 0;
    _everyInitPosition = 0;
    _flagStop = 1;
}
/**********************************************************
Description: BMP73T102 initial 
Parameters:                   
Return:             
Others: Pin set to output mode and low level   
**********************************************************/
void BMP73T102::begin()
{
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin3, OUTPUT);
    pinMode(_pin4, OUTPUT);
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    digitalWrite(_pin3, LOW);
    digitalWrite(_pin4, LOW);
}
/******************************************************************************
Description: Rotating DC motor  
Parameters:  ch: DC motor number:
                                  1: DC motor 1
                                  2: DC motor 2
             rank: Speed gear,range -100~100,positive is clockwise, 
                   negative is counterclockwise, absolute value indicates gear     
Return:             
Others:      
******************************************************************************/
void BMP73T102::dcMotorRun(uint8_t ch, int8_t rank)
{  
    if (_interface==0)
    {
        rank = constrain(rank, -100, 100);
        int a = abs(rank);
        int _rank = map(a, 0, 100, 0, 255);
        if (ch == DC1)
        {
            if (rank >= 0)      
            {
                pinMode(6, OUTPUT);
                digitalWrite(6, LOW);             
                analogWrite(9, _rank);
            } 
            else
            {
                pinMode(9, OUTPUT);
                digitalWrite(9, LOW);             
                analogWrite(6, _rank);  
            }
            _rank1 = rank;
        }
        else if (ch == DC2)
        {
            if (rank >= 0)      
            {
                pinMode(3, OUTPUT);
                digitalWrite(3, LOW);             
                analogWrite(5, _rank);
            } 
            else
            {
                pinMode(5, OUTPUT);
                digitalWrite(5, LOW);             
                analogWrite(3, _rank);  
            }
            _rank2 = rank;
        }
    }
}
/**********************************************************
Description: DC motor stop  
Parameters:  ch: DC motor number:
                                  1: DC motor 1
                                  2: DC motor 2                   
Return:             
Others:      
**********************************************************/
void BMP73T102::dcMotorStop(uint8_t ch)
{
    if (_interface==0)
    {
        if (ch == DC1)
        {
            pinMode(9, OUTPUT);
            pinMode(6, OUTPUT);
            digitalWrite(9, LOW);
            digitalWrite(6, LOW);
        }
        if (ch == DC2)
        {
            pinMode(5, OUTPUT);
            pinMode(3, OUTPUT);
            digitalWrite(5, LOW);
            digitalWrite(3, LOW);
        }   
    }
}
/**********************************************************
Description: DC motor brake  
Parameters:  ch: DC motor number:
                                  1: DC motor 1
                                  2: DC motor 2                 
Return:             
Others:      
**********************************************************/
void BMP73T102::dcMotorBrake(uint8_t ch)
{
    if (_interface==0)
    {
        if (ch == DC1)
        {
            pinMode(9, OUTPUT);
            pinMode(6, OUTPUT);
            digitalWrite(9, HIGH);
            digitalWrite(6, HIGH);
        }
        if (ch == DC2)
        {
            pinMode(5, OUTPUT);
            pinMode(3, OUTPUT);
            digitalWrite(5, HIGH);
            digitalWrite(3, HIGH);
        }  
    } 
}
/************************************************************************
Description: Get DC motor speed gear  
Parameters:  ch: DC motor number:
                                  1: DC motor 1
                                  2: DC motor 2               
Return:      DC motor speed gear: -100~100    
Others:      
************************************************************************/
int8_t BMP73T102::getDcMotorRank(int8_t ch)
{
    if (_interface==0)
    {
        if (ch == DC1)
        {
            return(_rank1);
        }
        if (ch == DC2)
        {
            return(_rank2);
        } 
        return(0);
    }
    else 
    {
        return(0);
    }   
}
/**********************************************************
Description: The stepper motor rotates to the absolute position at a constant speed  
Parameters:  absolute: Absolute position,a position relative to the zero position,steps(unit)
             speed: The speed of the stepper motor,steps per second(unit)  
                    It is recommended not to exceed 800 steps/s under full step
                    It is recommended not to exceed 1600 steps/s under half step                    
Return:               
Others:      This function runs the "block" program. That is, Arduino will not
             continue to execute the subsequent program content until the target location is reached.     
**********************************************************/
void BMP73T102::stepperMoveTo(int32_t  absolute, uint16_t speed)
{  
    if (_interface)
    {
        if (_targetPos != absolute)
        {   
            _flagStop = 0;
            _flagAccelerate = 0;  
            _targetPos = absolute;
            _dir = (_targetPos - _currentPos > 0) ? FORWARD : REVERSAL; 
      
        }  
         setSpeed(speed);
         stepperKeepRun(); 
    }
}
/**********************************************************
Description: The stepper motor rotates to the absolute position with acceleration      
Parameters:  absolute: Absolute position,a position relative to the zero position,steps(unit)                     
Return:                
Others:      This function runs the "block" program. That is, Arduino will not
             continue to execute the subsequent program content until the target location is reached.
**********************************************************/
void BMP73T102::stepperMoveTo(int32_t absolute)
{
    if (_interface)
    {
        if (_targetPos != absolute )
        {
            _flagStop = 0;
            _targetPos = absolute;
            _flagAccelerate = 1;
            computeNewSpeed();
            stepperKeepRun();
        }
    }
}
/**********************************************************
Description: How many steps does the stepper motor rotate at a constant speed    
Parameters:  relative: Rotation steps of stepper motor, positive indicates clockwise, 
                       negative indicates counterclockwise,steps(unit)
             speed: The speed of the stepper motor,steps per second(unit)                      
Return:                
Others:      This function runs the "block" program. That is, Arduino will not
             continue to execute the subsequent program content until the target location is reached.
**********************************************************/
void BMP73T102::stepperMove(int32_t relative, uint16_t speed)
{
    if (_interface)
    {
        stepperMoveTo(_currentPos + relative, speed);
    }
}
/**********************************************************
Description: How many steps does the stepping motor rotate with acceleration     
Parameters:  relative:Rotation steps of stepper motor, positive indicates clockwise, 
                      negative indicates counterclockwise,steps(unit)                   
Return:               
Others:      This function runs the "block" program. That is, Arduino will not
             continue to execute the subsequent program content until the target location is reached.
**********************************************************/
void BMP73T102::stepperMove(int32_t  relative)
{
    if (_interface)
    {
        stepperMoveTo(_currentPos + relative);
    }
}
/**********************************************************
Description: Get maximum speed of stepper motor         
Parameters:   
Return:      Maximum speed of stepper motor,steps per second(unit)      
Others:      
**********************************************************/
uint16_t BMP73T102::getStepperMaxSpeed()
{
    if (_interface)
    {
        return _maxSpeed;
    }
    else 
    {
        return(0);
    }    
}
/**********************************************************
Description: Get acceleration of stepper motor       
Parameters:  
Return:      acceleration of stepper motor,step per square second(unit)      
Others:      
**********************************************************/
uint16_t BMP73T102::getStepperAcceleration()
{
    if (_interface)
    {
        return _acceleration;
    }
    else 
    {
        return(0);
    }    
}
/**********************************************************
Description: Get the current position of stepper motor        
Parameters:   
Return:      The current position of stepper motor,steps(unit)     
Others:      
**********************************************************/
int32_t BMP73T102::getStepperPosition()
{
    if (_interface)
    {
        return _currentPos;
    }
    else 
    {
        return(0);
    }    
}
/**********************************************************
Description: Set the maximum speed of the stepper motor  
Parameters:  maxSpeed: The maximum speed of the stepper motor,     
                       steps per second(unit)   
                       It is recommended not to exceed 800 steps/s under full step
                       It is recommended not to exceed 1600 steps/s under half step                
Return:             
Others:            
**********************************************************/
void BMP73T102::setStepperMaxSpeed(uint16_t maxSpeed)
{
    if (_interface)
    {
        if (_maxSpeed != maxSpeed)
        {
           
            _maxSpeed = (float)maxSpeed;
            _minStepInterval = 1000000 / maxSpeed;      
            if (_count > 0)   // Recompute _count from current speed and adjust speed if accelerating or cruising
            {
                _count = (long)((_speed * _speed) / (2 * _acceleration)); 
                computeNewSpeed();
            }
        }
    }
}
/**********************************************************
Description: Set the acceleration of the stepper motor  
Parameters:  acceleration: The acceleration of the stepper motor,step per square second(unit)                       
Return:             
Others:      
**********************************************************/
void BMP73T102::setStepperAcceleration(uint16_t acceleration)
{
    if (_interface)
    {
        if (acceleration == 0)
        {
            return;
        }
        if (_acceleration != acceleration)
        {
            _count = _count * (_acceleration / ((float)acceleration));  // Recompute _count  
            _initStepInterval = 0.676 * sqrt(2 / ((float)acceleration)) * 1000000; // New c0
            _acceleration = (float)acceleration;
        }
    }
}
/**********************************************************
Description: Set the current position of stepping motor           
Parameters:  position: Position of stepper motor,steps(unit)                     
Return:                
Others:      
**********************************************************/
void BMP73T102::setStepperCurrentPosition(int32_t position)
{
    if (_interface)
    {
        _targetPos = _currentPos = position;
        _count = 0;
        _stepInterval = 0;
        _speed = 0;
        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, LOW);
        digitalWrite(_pin3, LOW);
        digitalWrite(_pin4, LOW);  
    }      
}
/**********************************************************
Description: Set the speed of the stepper motor  
Parameters:  speed:The speed of the stepper motor,steps per second(unit)                             
Return:             
Others:      It requires less than the maximum speed
**********************************************************/
void BMP73T102::setSpeed(uint16_t speed)
{
    if (_interface)
    {
        if (speed == _speed)
        {
            return;
        }
        speed = constrain(speed, -_maxSpeed, _maxSpeed);
        if (speed == 0)
        {
            _stepInterval = 0;
        }
        else
        {
            _stepInterval = fabs(1000000 / speed);
        }
        _speed = fabs(speed);
        _speed=(float)_speed;
    }
}
/**********************************************************
Description: Calculate the new instantaneous speed and set it to current speed       
Parameters:                                  
Return:                
Others:      
**********************************************************/
void BMP73T102::computeNewSpeed()
{
    if (_interface)
    {
        _distanceTo = getStepperDistanceToGo(); 
        long stepsToStop = (long)((_speed * _speed) / (2 * _acceleration)); 
        if (_distanceTo == 0 && stepsToStop <= 1)
        {
            _nextStepInterval = _nextStepInterval - ((2 * _nextStepInterval) / ((4 * _count) + 1)); 
            _nextStepInterval = max(_nextStepInterval, _minStepInterval); 
            _count++;
            _stepInterval = _nextStepInterval;
            _speed = 1000000 / _nextStepInterval;
            if (_dir == REVERSAL)
           {
               _speed = -_speed;
           }  
            return;
        }
        if (_distanceTo > 0) // Need to go clockwise from here, maybe decelerate now
        {      
            if (_count > 0) // Currently accelerating
            {  
                if ((stepsToStop >= _distanceTo) || _dir == REVERSAL) //Need to decel now?
                _count = -stepsToStop; // Start deceleration
            }
            else if (_count < 0)       // Currently decelerating
            {           
                if ((stepsToStop < _distanceTo) && _dir == FORWARD)   //Need to accel again?
                _count = -_count; // Start accceleration
            }
        }
        else if (_distanceTo < 0) // Need to go anticlockwise from here, maybe decelerate
        {       
            if (_count > 0) // Currently accelerating
            {
                if ((stepsToStop >= -_distanceTo) || _dir == FORWARD) //Need to decel now?
                _count = -stepsToStop; // Start deceleration
            }
            else if (_count < 0)       // Currently decelerating
            {
                if ((stepsToStop < -_distanceTo) && _dir == REVERSAL) //Need to accel again?
                _count = -_count; // Start accceleration
            }
        }  
        if (_count == 0) // Need to accelerate or decelerate
        {    
            _nextStepInterval = _initStepInterval;   // First step from stopped
            _dir = (_distanceTo > 0) ? FORWARD : REVERSAL;
        }
        else    // Subsequent step. Works for accel and decel
        {
            _nextStepInterval = _nextStepInterval - ((2 * _nextStepInterval) / ((4 * _count) + 1)); 
            _nextStepInterval = max(_nextStepInterval, _minStepInterval); 
        }
        _count++;
        _stepInterval = _nextStepInterval;
        _speed = 1000000 / _nextStepInterval;
        if (_dir == REVERSAL)
        {
            _speed = -_speed;
        }
    }
}
/**********************************************************
Description: Stepping function and half stepping function of stepper motor         
Parameters:  step: Running steps of stepper motor,steps(unit)                    
Return:               
Others:      
**********************************************************/
void BMP73T102::step (long step)
{
    if (_interface == FULL4WIRE)//Full step drive
    {
        switch (step & 0x3)
        {
            case 0://1011
                digitalWrite(_pin1, HIGH);    
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, HIGH);
                digitalWrite(_pin4, HIGH);
                break;
            case 1://1110
                digitalWrite(_pin1, HIGH);    
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, HIGH);
                digitalWrite(_pin4, LOW);
                break;
            case 2://0111
                digitalWrite(_pin1, LOW);    
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, HIGH);
                digitalWrite(_pin4, HIGH);
                break;
            case 3://1101
                digitalWrite(_pin1, HIGH);    
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, LOW);
                digitalWrite(_pin4, HIGH);
                break;
        }
    }
    else//Half step drive  
    {       
        switch (step & 0x7)
        {
            case 0://1000
                digitalWrite(_pin1, HIGH);    
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, LOW);
                digitalWrite(_pin4, LOW);
                break;
            case 1://1010
                digitalWrite(_pin1, HIGH);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, HIGH);                
                digitalWrite(_pin4, LOW);
                break;
            case 2://0010
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, HIGH);        
                digitalWrite(_pin4, LOW);
                break;
            case 3://0110
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, HIGH);        
                digitalWrite(_pin4, LOW); 
                break;
            case 4://0100
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, LOW);                
                digitalWrite(_pin4, LOW);
                break;
            case 5://0101
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, LOW);                
                digitalWrite(_pin4, HIGH);
                break;
            case 6://0001
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, LOW);                
                digitalWrite(_pin4, HIGH);
                break;
            case 7://1001
                digitalWrite(_pin1, HIGH);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, LOW);                
                digitalWrite(_pin4, HIGH);
                break;          
        }
    }
}
/**********************************************************
Description: Stepper motor runs one step   
Parameters:                           
Return:      true:Success
             false:Failed       
Others:      
**********************************************************/
boolean BMP73T102::run()
{   
    if (_interface)
    { 
        if (!_stepInterval)  // Dont do anything unless we actually have a step interval
        {
            return false;
        }
        unsigned long time = micros();   
        if (time - _lastStepTime >= _stepInterval)
        {
            if (_dir == FORWARD)
            {
                _currentPos += 1;  // FORWARD
            }
            else
            {
                _currentPos -= 1;  // REVERSAL  
            } 
            step(_currentPos);    
            _lastStepTime = time; // Caution: does not account for costs in step()
            return true;
        }
        else
        {
            return false;
        }
    }
    else 
    {
        return(0);
    }    
} 
/**********************************************************
Description: Stepper motor rotation     
Parameters:                       
Return:           
Others:     
**********************************************************/
void BMP73T102::stepperKeepRun()
{   
    if (_interface)
    {
        _distanceTo = getStepperDistanceToGo();
        while (_flagStop == 0 && _flagAccelerate == 0) //The stepper motor rotates at a constant speed
        {
            if (run())
            {
                _distanceTo = getStepperDistanceToGo();
            } 
            else
            {
                if (getStepperPosition() != _everyInitPosition)
                {
                    if (_interface == HALF4WIRE)
                    {
                        step(_currentPos);
                        if (fabs(_speed) < 800)
                        {
                            delayMicroseconds(25); 
                            digitalWrite(_pin1, HIGH);
                            digitalWrite(_pin2, HIGH);
                            digitalWrite(_pin3, HIGH);
                            digitalWrite(_pin4, HIGH);
                        }
                        else
                        {
                            delayMicroseconds(80);
                            digitalWrite(_pin1, HIGH);
                            digitalWrite(_pin2, HIGH);
                            digitalWrite(_pin3, HIGH);
                            digitalWrite(_pin4, HIGH);
                        }
                    }            
                }
            }
            if (_distanceTo == 0)     
            {
                while (1)
                {                    
                    unsigned long time1 = micros();   
                    if ((time1 - _lastStepTime >= _stepInterval) && (time1 - _lastStepTime < 5*_stepInterval))  //Maintain five steps to release current
                    {        
                        digitalWrite(_pin1, HIGH);
                        digitalWrite(_pin2, HIGH);
                        digitalWrite(_pin3, HIGH);
                        digitalWrite(_pin4, HIGH);
                    }
                    else if (time1 - _lastStepTime >= 5*_stepInterval)
                    {
                        _lastStepTime = time1;
                        digitalWrite(_pin1, LOW);
                        digitalWrite(_pin2, LOW);
                        digitalWrite(_pin3, LOW);
                        digitalWrite(_pin4, LOW);
                        _flagStop = 1;
                        _speed = 0;
                        _stepInterval = 0 ;
                        _everyInitPosition = getStepperPosition();         
                        break; 
                    }
                    else
                    {
                        if (_interface == HALF4WIRE)
                        {
                            step(_currentPos);
                            if (fabs(_speed) < 800)
                            {
                                delayMicroseconds(25); 
                                digitalWrite(_pin1, HIGH);
                                digitalWrite(_pin2, HIGH);
                                digitalWrite(_pin3, HIGH);
                                digitalWrite(_pin4, HIGH);
                            }
                            else
                            {
                                delayMicroseconds(80);
                                digitalWrite(_pin1, HIGH);
                                digitalWrite(_pin2, HIGH);
                                digitalWrite(_pin3, HIGH);
                                digitalWrite(_pin4, HIGH);
                            }  
                        }
                    }
                }
                break;
            }
        }     
        while (_flagStop == 0 && _flagAccelerate == 1) //The stepper motor rotates at an acceleration
        {      
            if (run())
            {
                computeNewSpeed();
            }
            else  
            {
                if (getStepperPosition() != _everyInitPosition)
                {
                    if (_interface == HALF4WIRE)
                    {
                        step(_currentPos);
                        if (fabs(_speed) < 800)
                        {
                            delayMicroseconds(25); 
                        }
                        else
                        {
                            delayMicroseconds(80);
                        }
                        digitalWrite(_pin1, HIGH);
                        digitalWrite(_pin2, HIGH);
                        digitalWrite(_pin3, HIGH);
                        digitalWrite(_pin4, HIGH);
                    }           
                }
            }
            if (_distanceTo == 0)
            {     
                while (1)
                {                 
                    unsigned long time2 = micros();   
                    if ((time2 - _lastStepTime >= _stepInterval) && (time2 - _lastStepTime < 5*_stepInterval)) //Maintain five steps to release current
                    {
                        digitalWrite(_pin1, HIGH);
                        digitalWrite(_pin2, HIGH);
                        digitalWrite(_pin3, HIGH);
                        digitalWrite(_pin4, HIGH);
                    }
                    else if (time2 - _lastStepTime >= 5*_stepInterval)
                    {
                        _lastStepTime = time2;
                        digitalWrite(_pin1, LOW);
                        digitalWrite(_pin2, LOW);
                        digitalWrite(_pin3, LOW);
                        digitalWrite(_pin4, LOW);
                        _flagStop = 1;
                        _flagAccelerate = 0;
                        _speed = 0;
                        _stepInterval = 0 ;
                        _count = 0;
                        _everyInitPosition = getStepperPosition();         
                        break; 
                    }
                    else
                    {
                        if (_interface == HALF4WIRE)
                        {
                            step(_currentPos);
                            if (fabs(_speed) < 800)
                            {
                                delayMicroseconds(20); 
                            }
                            else
                            {
                                delayMicroseconds(80);
                            }
                            digitalWrite(_pin1, HIGH);
                            digitalWrite(_pin2, HIGH);
                            digitalWrite(_pin3, HIGH);
                            digitalWrite(_pin4, HIGH);
                        }   
                    }
                }
                break;
            }
        }
    }
}
/**********************************************************
Description: Get the distance between the current position of the stepper motor and the target position        
Parameters:   
Return:      The distance between the current position of the stepper motor and the target position,steps(unit)      
Others:      
**********************************************************/
int32_t BMP73T102::getStepperDistanceToGo()
{
    if (_interface)
    {
        return _targetPos - _currentPos;
    }
    else 
    {
        return(0);
    }    
}
