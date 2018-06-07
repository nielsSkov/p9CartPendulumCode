/*
  Utility.cpp - Library for extra math functions.
  Created by Rasmus Pedersen, July 10, 2014.
*/

#include "Arduino.h"
#include "Utility.h"


// Sign function.
int sign(float x){
    if (x<0) { return-1; }
    if (x==0) {return 0; }
    return 1;
}

float sat(float x, float eps) {
    if (abs(x) > 1) {
        return sign(x);
    }
    else {
        return x*(1/eps);
    }
}
