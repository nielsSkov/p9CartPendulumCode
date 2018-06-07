/*
  EKF.h - Library for Extended Kalman Filter. Utilizes the Eigen library. Created by GRP 1030, spring 2017.
*/
#ifndef EKF_h
#define EKF_h

#include "Arduino.h"
#include <stdbool.h>
#include <paramsGR171030.h>
#include <Eigen312.h> 
#include <LU>   

class EKF
{
  public:
    // Constructor. Initializes matricies.
    EKF(float Ts, float start_x, float start_th);
    
    // Main EKF function. Returns estimated states. Input in SI units.
    void getStateEstimates(float states[], float x, float th, float u);

    // Reset mr Kalman
    void resetEKF(boolean reset);

    // Internal matrix functions
    Eigen::Vector4f f(const Eigen::Vector4f& xEstkk, const Eigen::Vector2f& uk);
    Eigen::Matrix4f numjacF(Eigen::Vector4f& xEstkk, const Eigen::Vector2f& uk);
    Eigen::Matrix4f jacF(Eigen::Vector4f& xEstkk);

    Eigen::Matrix2f Minvmat(float th);
    Eigen::Vector2f Gmat(float th);
    Eigen::Vector2f Cmat(float th, float thd);
    Eigen::Vector2f Bmat(float xd, float thd);

    Eigen::Matrix2f MatInv(const Eigen::Matrix2f& _mat);
    Eigen::MatrixXf MatTrans(const Eigen::MatrixXf& _mat);
  private:
    // Physical constants
    boolean _reset;

    float _g;
    float _mc;
    float _mp;
    float _lRod;
    float _lMass;
    float _l;
    float _ticksPerRev;
    float _ticksPerRad;
    float _ticksPerM;
    float _Ts;
    float _pi;
    // For jacobian F
    float _eps;
    float _epsInv;
    float _estimatedStates[4];

    // Frictions coefficients
    float _cx;
    float _cxL;
    float _cxR;
    float _k;
    float _cth;
    float _vth;

    // Matricies used for kalman calculations
    // [4x4]
    Eigen::Matrix4f Pkkm1;
    Eigen::Matrix4f Pkp1k;
    Eigen::Matrix4f Pkk;
    Eigen::Matrix4f Pkk_tmp;
    Eigen::Matrix4f Fk;
    Eigen::Matrix4f Qk;
    Eigen::Matrix4f I;

    // [4x1]
    Eigen::Vector4f xEstkk;
    Eigen::Vector4f xEstkkm1;
    Eigen::Vector4f xEstkp1k;

    // [4x2]
    Eigen::MatrixXf Kk;
    Eigen::MatrixXf Hk_T;

    // [2x2]
    Eigen::Matrix2f Rk;
    Eigen::Matrix2f Kk_tmp;
    
    // [2x4]
    Eigen::MatrixXf Hk;

    // [2x1]
    Eigen::Vector2f yk;
    Eigen::Vector2f yEstkkm1;
    Eigen::Vector2f yReskkm1;

    Eigen::Vector2f G;
    Eigen::Vector2f C;
    Eigen::Vector2f B;
    Eigen::Matrix2f Minv;
    Eigen::Vector2f Fsum;
    Eigen::Vector2f accs;
    Eigen::Vector2f uk;
    Eigen::Vector4f tmp;
    Eigen::Matrix4f jacFout;

};

#endif
