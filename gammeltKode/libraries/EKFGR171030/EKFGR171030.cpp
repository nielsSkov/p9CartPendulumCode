/*
  EKF.cpp - Library for Extended Kalman Filter algorithm.
  Created by Jesper Hede Christensen and Rasmus Christiansen, GRP1030, spring 2017.
*/

#include "Arduino.h"
#include "EKFGR171030.h"

// Constructor. Initialize matricies and constants
EKF::EKF(float Ts, float start_x, float start_th)
{
	_reset = false;

	// Compare to params.m
    _pi     = M_PI;
	_g		= g;
	_mc		= mc;
	_mp		= mp;
	_lRod	= 28.25e-2;
	_lMass	= 10.4e-2;
    _l 		= _lRod+_lMass/2;
    _ticksPerRev = 2000;
    _ticksPerM = 8737/(769/1e3);
    _ticksPerRad = _ticksPerRev/(2*_pi);
    _Ts = Ts;

    // For numerical jacobian F
    _eps 	= 1e-6;
    _epsInv = 1/_eps;

    // Friction coefficients
    _cx		= 0;
    _cxL	= cxL;
	_cxR	= cxR;
    _k		= 500;
	_cth	= cth;
	_vth	= vth;

	/* ------ Define measurement and state noise ------ */
	Qk = Eigen::Matrix4f::Zero();
	Rk = Eigen::Matrix2f::Zero();

	Qk(0,0) = 1;		// x
	Qk(1,1) = 1;	  // th
	Qk(2,2) = 50;				// xd
	Qk(3,3) = 50;				// thd

	Rk(0,0) = 0.01;				// x
	Rk(1,1) = 0.1;				// th
	/* ------------------------------------------------ */

	// [4x4]
	Pkkm1 = Eigen::Matrix4f::Zero();
    Pkp1k = Eigen::Matrix4f::Zero();
    Pkk = Eigen::Matrix4f::Zero();
    Pkk_tmp = Eigen::Matrix4f::Zero();
    Fk = Eigen::Matrix4f::Zero();
    I = Eigen::Matrix4f::Identity(4,4);

    // [4x1]
    xEstkk = Eigen::Vector4f::Zero();
    xEstkkm1 = Eigen::Vector4f::Zero();
    xEstkp1k = Eigen::Vector4f::Zero();

    // Initialize filter states
    xEstkp1k(0) = start_x;
    xEstkp1k(1) = start_th;      
    xEstkp1k(2) = 0;
    xEstkp1k(3) = 0;


    // [4x2]
    Kk = Eigen::MatrixXf::Zero(4,2);
    
    // [2x2]
    Kk_tmp = Eigen::Matrix2f::Zero();
    
    // [2x4]
    Hk = Eigen::MatrixXf::Zero(2,4);
    Hk(0,0) = 1;
    Hk(1,1) = 1; 
    Hk_T = Hk.transpose();

    // [2x1]
    yk = Eigen::Vector2f::Zero();
    yEstkkm1 = Eigen::Vector2f::Zero();
    yReskkm1 = Eigen::Vector2f::Zero();

    G = Eigen::Vector2f::Zero();
    B = Eigen::Vector2f::Zero();
    C = Eigen::Vector2f::Zero();
    Fsum = Eigen::Vector2f::Zero();
    uk = Eigen::Vector2f::Zero();
    accs = Eigen::Vector2f::Zero();
    Minv = Eigen::Matrix2f::Zero();
    tmp = Eigen::Vector4f::Zero();
    jacFout = Eigen::Matrix4f::Zero();
}

void EKF::resetEKF(boolean reset){
	if(reset)
		_reset = true;
	else
		_reset = false;
}

// Main EKF function. Returns estimated states. Input in SI units.
void EKF::getStateEstimates(float states[], float x, float th, float u)
{
	if(_reset){
		xEstkp1k 	= Eigen::Vector4f::Zero();
		Pkp1k 		= Eigen::Matrix4f::Zero();
		_reset = false;
	}

	// Get measurements and control action
	yk(0) = x; 		// cart position measurement
    yk(1) = th;		// pendulum position measurement
  	uk(0) = u;		// control action

    // Update from previous time step
    xEstkkm1 = xEstkp1k;
    Pkkm1 = Pkp1k;

    // Estimate output given (input) and previous state estimate
    yEstkkm1(0) = xEstkkm1(0);
    yEstkkm1(1) = xEstkkm1(1);

    // Find residual from actual measurement
    yReskkm1 = yk - yEstkkm1;

    // Kalman gain
	Kk = Pkkm1 * Hk_T * MatInv(Hk * Pkkm1 * Hk_T + Rk); 
    
    // Compute current estimate
    xEstkk = xEstkkm1 + Kk*yReskkm1; 

    // Compute covariance matrix
    Pkk_tmp = I - Kk*Hk;
    Pkk = Pkk_tmp * Pkkm1 * MatTrans(Pkk_tmp) + Kk*Rk*MatTrans(Kk);
    
    /* ----- Time Update ----- */ 
    // Find state estimate for next step
    xEstkp1k = f(xEstkk, uk);

    // Linearize and transpose for covariance matrix 
    Fk = jacF(xEstkk);
    Pkp1k = Fk*Pkk*MatTrans(Fk) + Qk;

    states[0] = xEstkk(0);
    states[1] = xEstkk(1);
    states[2] = xEstkk(2);
    states[3] = xEstkk(3);
    states[4] = tmp(1);		// debug output
}

Eigen::Vector4f EKF::f(const Eigen::Vector4f& xEstkk, const Eigen::Vector2f& uk)
{	/* 
		xEstkk(0) = x
		xEstkk(1) = th
		xEstkk(2) = xd
		xEstkk(3) = thd
	*/
	//float x   = xEstkk(0);     // Unused
	float th  = xEstkk(1);
	float xd  = xEstkk(2);
	float thd = xEstkk(3);

    // Gravitational effects 
    G(0) = 0;
    G(1) = _mp *_l * _g * sin(th);
    
    // Centrifugal effect
    C(0) = -_mp * _l * sin(th) * thd * thd;
    C(1) = 0;
    
    if(xd > 0)	
    	_cx = _cxR;
    else if(xd < 0) 
    	_cx = _cxL;
    else
    	_cx = 0;

    // Friction
    B(0) = -( _cx*tanh(_k*xd) );
    B(1) = -( _cth*tanh(_k*thd) + _vth*thd);  
    
    // Inertial properties
    Minv(0,0) = 1/(_mc+_mp - _mp*cos(th)*cos(th));
    Minv(0,1) = cos(th)/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th));
    Minv(1,0) = cos(th)/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th));
    Minv(1,1) = (_mp + _mc)/(_l*_l*_mp*_mp + _l*_l*_mc*_mp - _l*_l*_mp*_mp*cos(th)*cos(th));

    accs = Minv * (uk + G + C + B);

// Uses approx 100 µs more.. 
//    Fsum = uk + Gmat(th) + Cmat(th, thd) + Bmat(xd, thd);
//   accs = Minvmat(th) * Fsum;

	tmp(0) = xd;
	tmp(1) = thd;
	tmp(2) = accs(0);
	tmp(3) = accs(1);

	return xEstkk + _Ts * tmp;
}

Eigen::Matrix4f EKF::numjacF(Eigen::Vector4f& xEstkk, const Eigen::Vector2f& uk)
{
	Eigen::Vector4f f0 = f(xEstkk, uk);
	Eigen::Vector4f fx = Eigen::Vector4f::Zero();

	for(int i = 0; i < 4; i++){
		xEstkk(i) = xEstkk(i) + _eps;
		fx = f(xEstkk, uk);

		jacFout.col(i) = (fx - f0) * _epsInv;
	}

	return jacFout; 
}

Eigen::Matrix4f EKF::jacF(Eigen::Vector4f& xEstkk)
{
    Eigen::Matrix4f gradaccs = Eigen::Matrix4f::Zero();

    float th  = xEstkk(1);
    float thd = xEstkk(3);

    // Singularity fix
    if(cos(th) < 1e3){
        int sign = (th > 0) ? 1 : -1; // Determine sign
        th = sign*(th+1e-3);
    }

    // Mega Matrix
    gradaccs(0,2) = 1;
    gradaccs(1,3) = 1;
    gradaccs(2,1) = (_l*_mp*thd*thd*cos(th))/(_mc + _mp - _mp*cos(th)*cos(th)) - (_g*_l*_mp*cos(th)*cos(th))/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th)) + (_g*_l*_mp*sin(th)*sin(th))/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th)) - (2*_l*_mp*_mp*thd*thd*cos(th)*sin(th)*sin(th))/((- _mp*cos(th)*cos(th) + _mc + _mp)*(- _mp*cos(th)*cos(th) + _mc + _mp)) + (2*_g*_l*_l*_mp*_mp*cos(th)*cos(th)*sin(th)*sin(th))/((- _l*_mp*cos(th)*cos(th) + _l*_mc + _l*_mp)*(- _l*_mp*cos(th)*cos(th) + _l*_mc + _l*_mp));
    gradaccs(3,1) = (_l*_mp*thd*thd*cos(th)*cos(th))/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th)) - (_l*_mp*thd*thd*sin(th)*sin(th))/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th)) - (_g*_l*_mp*cos(th)*(_mc + _mp))/(_l*_l*_mp*_mp + _l*_l*_mc*_mp - _l*_l*_mp*_mp*cos(th)*cos(th)) - (2*_l*_l*_mp*_mp*thd*thd*cos(th)*cos(th)*sin(th)*sin(th))/((- _l*_mp*cos(th)*cos(th) + _l*_mc + _l*_mp)*(- _l*_mp*cos(th)*cos(th) + _l*_mc + _l*_mp)) + (2*_g*_l*_l*_l*_mp*_mp*_mp*cos(th)*sin(th)*sin(th)*(_mc + _mp))/((- _l*_l*_mp*_mp*cos(th)*cos(th) + _l*_l*_mp*_mp + _mc*_l*_l*_mp)*(- _l*_l*_mp*_mp*cos(th)*cos(th) + _l*_l*_mp*_mp + _mc*_l*_l*_mp));
    gradaccs(2,3) = (2*_l*_mp*thd*sin(th))/(_mc + _mp - _mp*cos(th)*cos(th));
    gradaccs(3,3) = (2*_l*_mp*thd*cos(th)*sin(th))/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th));

    return I + _Ts * gradaccs;
}


Eigen::Matrix2f EKF::Minvmat(float th)
{
	Minv(0,0) = 1/(_mc+_mp - _mp*cos(th)*cos(th));
	Minv(0,1) = cos(th)/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th));
	Minv(1,0) = cos(th)/(_l*_mc + _l*_mp - _l*_mp*cos(th)*cos(th));
	Minv(1,1) = (_mp + _mc)/(_l*_l*_mp*_mp + _l*_l*_mc*_mp - _l*_l*_mp*_mp*cos(th)*cos(th));
	return Minv;
}

Eigen::Vector2f EKF::Gmat(float th)
{
	G(0) = 0;
	G(1) = _mp *_l * _g * sin(th);
	return G;
}

Eigen::Vector2f EKF::Cmat(float th, float thd)
{
	C(0) = -_mp * _l * sin(th) * thd * thd;
	C(1) = 0;
	return C;
}

Eigen::Vector2f EKF::Bmat(float xd, float thd)
{
    if(xd > 0)	
    	_cx = _cxR;
    else if(xd < 0) 
    	_cx = _cxL;
    else
    	_cx = 0;

	B(0) = -( _cx*tanh(_k*xd) );
	B(1) = -( _cth*tanh(_k*thd)	+ _vth*thd);
	return B;
}

Eigen::Matrix2f EKF::MatInv(const Eigen::Matrix2f& _mat)
{
    Eigen::Matrix2f _tmpInv = Eigen::Matrix2f::Zero();

    // Calculate determinant
    float det = _mat(0,0) * _mat(1,1) - _mat(0,1) * _mat(1,0);

    // Switch appropriate indexes and sign
    _tmpInv << _mat(1,1), -_mat(0,1), -_mat(1,0), _mat(0,0);

    return 1/det * _tmpInv;
}

Eigen::MatrixXf EKF::MatTrans(const Eigen::MatrixXf& _mat)
{
    Eigen::MatrixXf _tmpTrans;

    for(int i = 0; i < _mat.rows(); i++){
        _tmpTrans.col(i) = _mat.row(i);
    }
    return _tmpTrans;
}
