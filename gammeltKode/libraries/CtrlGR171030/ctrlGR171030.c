/*
  Ctrl.cpp - Library for inverted pendulum control functions.
  Created by Jesper Hede Christensen and Rasmus Christiansen, GRP1030, spring 2017.
*/

#include "Arduino.h"
#include "ctrlGR171030.h"
#include "paramsGR171030.h"

// Make vars locally visible
static float x;
static float th;
static float xd;
static float thd;
static float thdd;

static int kickstart= 0;
boolean _reset = false;
boolean trig = false;

// Persistent variables for PID stab control
static float thOldError = 0;
static float thErrorInt = 0;
static float thErrorDif = 0;
static float posOldError = 0;
static float posErrorInt = 0;
static float posErrorDif = 0;

void ctrl(float ctrlOut[], float ref, float states[5], int swup, int stab){
	float u = 0.0;
	float xRef = ref;
	int swupmode = swup;
	int stabmode = stab;

	// Unwrap states
	x 	= states[0];
	th 	= states[1];
	xd 	= states[2];
	thd = states[3];
	thdd = states[4];
	
	// Wrap th
	float thW = wrapPmPi(th);

	// Check catch-trigger and choose control accordingly
	trig = trigger(thW, stabmode);
	if(trig){
		u = stabCtrl(thW, stabmode);
	}
	else{
		u = swupCtrl(swupmode, xRef);
	}
	u = cBoost(u);
	ctrlOut[0] = u;		// control output

	float E = mp*l*l*thd*thd/2 + mp*g*l*(cos(th)+1);
	ctrlOut[1] = E; 	// debug output

}

///// Subfunctions ////
// Swing-up controller
float swupCtrl(int mode, float ref){
	float E = 0;
	float xRef = ref;
	int swupmode = mode;
	float u, u1, u2;

	switch(swupmode){
		case 0: break; // Disabled
		case 1:{
			static const float kd 	= 5;
   			static const float kv  	= 8;
   			static const float ke  	= 0.5;
			static const float kx  	= 3;
			static const float ki	= 0.001;//0.045;
			
			static const float m11 = mc+mp;
			static const float m22 = mp*l*l;
			float m12 = -mp*l*cos(th); // = m21
			E = m11*xd*xd + m22*thd*thd + 2*m12*xd*thd; // Ekin
			E = E + mp*g*l*(cos(th)+1);

			u1 = (-kd*xd -kx*(x-xRef) - kv*((mp*sin(th)*(-l*pow(thd,2)+g*cos(th))-cos(th)*vth*thd/l)/(mc+mp*pow(sin(th),2))))/(ke*(E-Eref)+kv/(mc+mp*pow(sin(th),2)));
			
			static float eeInt; // persistent integral energy
			eeInt = eeInt + SAMPLINGTIME*(Eref-E);
			u2 = eeInt*ki*thd*cos(th);	

			u = u1 + u2;
			kickstart = 0;

			// Reset integral error;
			if(_reset){
				eeInt = 0;
				_reset = false;
			}
		}
		case 2:{
			// Matlab gains
			// ke = 0.5;
			// kx = 1;
			// kint = 0.02;
			static const float ke = 0.44;//0.44;
			static const float kx = 1;
			static const float kint = 0.02;//0.017;
			static float eeInt; // persistent integral energy
			eeInt = eeInt + SAMPLINGTIME*(Eref-E);
			E = mp*l*l*thd*thd/2 + mp*g*l*(cos(th)+1);
			u = ke*(Eref-E);
			u += kint*eeInt;
			u *= thd*cos(th);
			u += kx*(xRef-x);
			u = acc2f(u);
			
			kickstart = 0;

			// Reset integral error;
			if(_reset){
				eeInt = 0;
				_reset = false;
			}
			break;
			}
		case 3:{
			const float kx = 0.5;//0.005;
			const float umax = 1.3;
			float Ep = mp*g*l*(cos(th)+1)+mp*l*l*thd*thd/2;

			u = umax*sign((Eref-Ep)*thd*cos(th));
			u = u + kx*(xRef-x);
			u = acc2f(u);
		}
		case 4:{
			float kx = 1.3;
			float ke = 0.8;
			float Ep = mp*g*l*(cos(th)+1)+mp*l*l*thd*thd/2;

			u = ke*sign((Eref-Ep)*thd)*cos(th);
			u = u + kx*(xRef-x);
			u = acc2f(u);
			kickstart = 1;
		}

	}
	return u;
}

// Stability controller
float stabCtrl(float thW, int mode){
	float u = 0;
	int stabmode = mode;
	float xRef = 0.444;

	// Wrap theta
	th = thW;

	// Define stabilisation state vector
	float statesStab[4] = {x,thW,xd,thd};

	// Calculate dynamics without cart friction
	float temp	= mc + mp*sin(th)*sin(th);
	float alpha	= mp*l*sin(th)*thd*thd/temp;
	float beta	= cos(th)*(cth*tanh(1e3*thd) + vth*thd - mp*l*g*sin(th))/(l*temp);
	float gamma	= cos(th)*mp*l*sin(th)*thd*thd/(l*temp);
	float delta	= (mc+mp)*(cth*tanh(1e3*thd) + vth*thd - mp*l*g*sin(th))/(mp*l*l*temp);

	float Fx		= -alpha-beta;
	float Fth		= -gamma-delta;

	float Gx		= 1/temp;
	float Gth		= cos(th)/(l*temp);

	float refs[4] = {xRef, 0, 0, 0};
	switch(stabmode){
		case 0: break; // Disabled
		case 1: {// PID 
			// Calculate position errors
			posErrorInt = posErrorInt + SAMPLINGTIME*(xRef-x);
 			posErrorDif = ((xRef-x) - posOldError)/SAMPLINGTIME;

			// Cart control (= thRef)
			float thRef = -(50*(xRef-x) + 25*posErrorInt + 25*posErrorDif);

			// Calculate angle errors
			thErrorInt = thErrorInt + SAMPLINGTIME*(thRef-th);
			thErrorDif = ((thRef-th) - thOldError)/ SAMPLINGTIME;

			u = 112*(thRef-th) + 15* thErrorInt + 15* thErrorDif;

			// Update errors
			thOldError = thRef - th;
			posOldError = xRef - x;
		}
		case 2: {// LQR
			for(int a = 0; a < 4; a++ ){
				u = u + lqrK[a]*(refs[a]-statesStab[a]);
			}
		}
		case 3:{ // Sliding mode
			// Matlab gains = [10 -1 -2]
			float k1 = 12;//12;
			float k2 = -1.2;
			float k3 = -2.5;//-2.5;
			float beta0 = 0.5;//0.5;

			float S = thd + k1*th + k2*(xd*cos(th)/l - thd) + k3*(x-xRef);
			float Phi = (1-k2)*Fth + k2*Fx*cos(th)/l + k1*thd + (k3 - k2*sin(th)/l)*xd;
			float Gamma = (1-k2)*Gth + k2*Gx*cos(th)/l;

			float beta = abs(Phi/Gamma) + beta0;
			float v = -beta*sign(S);
			u = (-Phi + v)/Gamma;
		}
	}
	return u;
}

void resetCtrl(boolean reset){
	if(reset)
		_reset = true;
	else
		_reset = false;
}

// Catch trigger
boolean trigger(float angleW, int mode){
	int stabmode = mode;

	float Ecatch = 1.05*Eref;
	float Epend  = mp*g*l*(cos(th)+1) + mp*l*l*thd*thd/2;
	if(stabmode != 0 && abs(Eref-Epend) <= abs(Eref-Ecatch)){
		if(abs(angleW) <= catchAng){
			return true;
		}
	}
	else
		return false;
}

// +/- pi wrap of theta
float wrapPmPi(float angle){
	float lb = -M_PI;
	float ub = M_PI;
	float db = ub-lb;

	while(angle > ub)
		angle = angle - db;

	while(angle < lb)
		angle = angle + db;

	return angle;
}

// Cart acc. to force
float acc2f(float uIn){
	float uOut = 0;
	// Complete term
	//uOut = (mc+mp)*uIn - mp*l*cos(th)*thdd + mp*l*sin(th)*thd*thd;

	// Simplified term
	uOut = (mc+mp)*uIn;

	return uOut;
}

// Coulomb boost and output limit
float cBoost(float uIn){
	float uOut = 0;
	float eps = 0;
	
	if(uIn > eps){
	 	uOut = uIn + cxR;
	}
	else if(uIn < -eps){
	 	uOut = uIn - cxL;
	}
	else{ 
		uOut = uIn; 
	}
	
	// limitations
	if(uOut > fub){ 		uOut = fub; }
	else if(uOut < flb){ 	uOut = flb; }
	
	return uOut;
}

// Sign function
int sign(float u){
	if(u > 0)
		return 1;
	else if(u < 0)
		return -1;
	else 
		return 0;
}

// Saturation function
float sat(float u){
	if (abs(u) <= 1)
		return u;
	else if (abs(u) > 1)
		return sign(u);
}
