/*
  Joint.h - Library for reading input and setting output of joint on pendulum.
  Created by Rasmus Pedersen, July 8, 2014.

  Modified by GRP 1030, spring 2017
*/
#ifndef Joint_h
#define Joint_h

#include "Arduino.h"

class Joint
{
  public:
    Joint(int joint, float Ts);   // 1 = sled, 2 = pendulum 1, 3 = pendulum 2.
    void init();
    float readPos();
	  float readVel();
    void resetPos();
    float setOutput(float value); // Input value should be [N]
    int safetyStop();
  private:
	float _Ts;
	float _oldPos;
  float _F_to_u;
    int _pinOE;
    int _pinSEL;
    int _pinRST;
    int _dataBusStartPin;
    int _outputSEL;
    int _outputAct;
    byte readByte(int startPin);
};

#endif