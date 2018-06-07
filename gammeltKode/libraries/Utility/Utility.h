/*
  Utility.h - Library for extra math functions.
  Created by Rasmus Pedersen, July 10, 2014.
*/


#ifndef Utility
#define Utility

#ifdef __cplusplus
extern "C" {
#endif
    
    int sign(float x);
    float sat(float x, float eps);
    
#ifdef __cplusplus
}
#endif

#endif