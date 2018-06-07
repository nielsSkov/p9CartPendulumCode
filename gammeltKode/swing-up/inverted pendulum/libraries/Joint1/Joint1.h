/*
  Joint.h - Library for reading input and setting output of joint on pendulum.
  Created by Rasmus Pedersen, July 8, 2014.
*/
#ifndef Joint1_h
#define Joint1_h

#include "Arduino.h"

class Joint
{
  public:
    Joint(int joint, float Ts);   // 1 = sled, 2 = pendulum 1, 3 = pendulum 2.
    void init();
    float readPos();
	float readVel();
    void resetPos();
    void setOutput(int value);
    int safetyStop();
  private:
	float _Ts;
	float _oldPos;
    int _pinOE;
    int _pinSEL;
    int _pinRST;
    int _dataBusStartPin;
    int _outputSEL;
    byte readByte(int startPin);
};

#endif
