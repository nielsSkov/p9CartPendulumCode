/*
  Ctrl.h - Library for inverted pendulum control functions.
  Created by Rasmus Christiansen and Jesper Hede Christensen, April 2, 2017.
*/

#ifndef Ctrl_h
#define Ctrl_h

#ifdef __cplusplus
extern "C" {
#endif

#include "Arduino.h"
#include <stdbool.h>

void ctrl(float ctrlOut[], float ref, float states[5], int swup, int stab);

boolean trigger(float angleW, int mode);
float swupCtrl(int mode, float ref);
float stabCtrl(float angleW, int mode);
void resetCtrl(boolean reset);
float wrapPmPi(float angle);
float acc2f(float uIn);
float cBoost(float uIn);
int sign(float u);
float sat(float u);

#ifdef __cplusplus
}
#endif


#endif
