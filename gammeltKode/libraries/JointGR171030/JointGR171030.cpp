/*
  Joint.cpp - Library for reading position and setting output of joint on pendulum.
  Created by Rasmus Pedersen, July 8, 2014.

  Modified by GRP 1030, spring 2017.
*/

#include "Arduino.h"
#include "JointGR171030.h"

// Constructor
Joint::Joint(int joint,float Ts)
{
    // Conversion of Newton to bit value
    _F_to_u = 56.744;

    // Set pins for specific joint:
    // Input:
    //      joint, is the specific joint. 1 = sled, 2 = pendulum 1 and 3 = pendulum 2.
	//		Ts, is the systems sampling time [sec].
	_Ts = Ts;
    _pinSEL = 35;
    _dataBusStartPin = 40;
    switch (joint) {
        case 1:             // Sled
            _outputSEL = 1;
            _pinOE = 30;
            _pinRST = 28;
            break;
        case 2:             // Pendulum 1
            _outputSEL = 2;
            _pinOE = 29;
            _pinRST = 31;
            break;
        case 3:             // Pendulum 2
            _outputSEL = 3;
            _pinOE = 33;
            _pinRST = 32;
            break;
        default:
            Serial.println("ERROR: Wrong joint selected. Only 1,2,3 are available");
            break;
    }

}

// Input all that needs to run in setup function in arduino
void Joint::init(){
    // Setup pins on arduino
    digitalWrite(_pinOE,HIGH);
    pinMode(_pinOE,OUTPUT);
    pinMode(_pinRST,OUTPUT);
    pinMode(_pinSEL,OUTPUT);
	// Initialize var
	_oldPos = 0;

}

// Output velocity of joint
float Joint::readVel(){
	float vel;
	if (_outputSEL == 1){
		vel = (readPos()-_oldPos)/_Ts;
		_oldPos=readPos();
		// Uncommented since readPos returns in m
        //vel = 0.001*vel; // mm/s -> m/s
	}
	else {
		vel = (readPos()-_oldPos)/_Ts;
		_oldPos=readPos();
	}
	return vel;

}

// Reset quadrature counters
void Joint::resetPos()
{
    digitalWrite(_pinRST,HIGH);
    delay(50);
    digitalWrite(_pinRST,LOW);
    delay(50);
    digitalWrite(_pinRST,HIGH);
}

float Joint::readPos()  // Modified to output SI units (m and rad)
{
    // Variables
    byte hByte;                     // high byte
    byte lByte;                     // low byte
    int16_t pos;                    // position in signed integer value
    float posFloat;                 // position in "real" value.
    float sledLength = 769;         // length of sled in mm
    float pendConst  = 360;         // degrees pr revolution
    float pendTicksPrRev = 2000;    // ticks pr revolution of pendulums
    float sledTicksPrLen = 8737;    // ticks from start to end of sled
    
    // Get position data
    digitalWrite(_pinOE,LOW);
    digitalWrite(_pinSEL,LOW);
    hByte = readByte(_dataBusStartPin);
    digitalWrite(_pinSEL,HIGH);
    lByte = readByte(_dataBusStartPin);
    digitalWrite(_pinOE,HIGH);
    
    // Convert the two bytes to int and cast to float
    pos = hByte*256 + lByte;
    posFloat = (float)pos;

    if (_outputSEL == 1) {
        // Converted from mm to m
        posFloat = (posFloat/(sledTicksPrLen/sledLength))/1000.0;
    }
    else {
        posFloat = posFloat/(pendTicksPrRev/pendConst);
        // Converted from deg to rad. Pi offset added. Always initialize with pendulum hanging down.
        posFloat = (posFloat*(-1))*(PI/180.0) + PI;
    }
    return posFloat;
}

// Read byte from data bus
byte Joint::readByte(int startPin) {
    int i;
    int j = 0;
    int in;
    byte out = 0;
    for(i=startPin;i<startPin+8;i++) {
        in = digitalRead(i);
        if(in==0) {
            out &= ~(1 << j);
        }
        else if(in==1) {
            out |= (1 << j);
        }
        j++;  
    }
    return out;
}

float Joint::setOutput(float value)
{
    int offset_sled = 2000 - 47; // Zero current offset corrected
    int offset_pend = 2000;

    // Frictions set to zero. Managed in ctrl.c
    int columnFricLSled = 0; //200
	int columnFricRSled = 0; //270
    int columnFricPend1CCW = 0; //10
	int columnFricPend1CW = 0; //15
    
    // Convert input value from Newton to bit value
    value = value * _F_to_u;

    // _outputAct is inverted to fit the defined coordinate system
    switch (_outputSEL) {
        case 1:
            // Minimum case
            if (offset_sled + value <= 0) {
                _outputAct = 0;
                analogWrite(DAC0,_outputAct);
            }
            // Maximum case
            else if (offset_sled + value >= 4095) {
                _outputAct = 4095;
                analogWrite(DAC0,_outputAct);
            }
            else {
                // Negative direction
                if (value < 0) {
                    // Changed ".. + value" to ".. - value"
                    _outputAct = offset_sled - value - columnFricRSled;
                    analogWrite(DAC0,_outputAct);

                }
                // Position direction
                // Changed ".. + value" to ".. - value"
                else if (value > 0){
                    _outputAct = offset_sled - value + columnFricLSled;
                    analogWrite(DAC0,_outputAct);
                }
                // Default (= 0)
                else {
                    _outputAct = offset_sled;
                    analogWrite(DAC0,_outputAct);
                }
            }
            break;
        case 2:
            if (offset_pend + value <= 0) {
                analogWrite(DAC1,0);
            }
            else if (offset_pend + value >= 4095) {
                analogWrite(DAC1,4095);
            }
            else {
                if (value < 0) {
                    analogWrite(DAC1,offset_pend + value - columnFricPend1CW);
                }
                else if (value > 0){
                    analogWrite(DAC1,offset_pend + value + columnFricPend1CCW);
                }
                else {
                    analogWrite(DAC1,offset_pend);
                }
            }
            break;
        case 3:
            break;
            
        default:
            Serial.println("ERROR: Wrong output selected (pendulum 2 not yet implemented)"); // HACK
            break;
    }

    // Should invert and convert back to Newton
    return _outputAct;
}

// Safety functions
int Joint::safetyStop() {
    // Variables
    float marginSled = 70;
    float marginPend = 720;
    // Sled safety
    if (_outputSEL == 1 ) {
        if (readPos() > 769 - marginSled || readPos() < marginSled) {
            setOutput(0);
            return 1;
        }
    }
    // Pendulum safety
    else {
        if (readPos() > marginPend || readPos() < -marginPend) {
            setOutput(0);
            return 1;
        }

    }
    return 0;
}