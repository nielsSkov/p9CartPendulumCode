/*
  params.h - Parameter list.
  Created by Jesper Hede Christensen and Rasmus Christiansen, GRP1030, spring 2017.
*/

#ifndef params_h
#define params_h

// Define system constants
const float SAMPLINGTIME = 5e-3;
const float g = 9.81;
const float mp = 0.2510;
const float mc = 4.75;
const float l = 0.3345;

const float cxL = 2.8; 	//3 // 3.4
const float cxR = 3.3;	//3.5 // 2.45
const float cth = 0.0038;
const float vth = 0.00067;
const float fub = 37.36;  // force upper bound
const float flb = -34.25; // force lower bound

// CTRL constants
#define Eref 2*mp*g*l
const float catchAng = 30*M_PI/180;

// CTRL global variables
const float lqrK[4] = {-750, 960, -10, 100};

#endif
