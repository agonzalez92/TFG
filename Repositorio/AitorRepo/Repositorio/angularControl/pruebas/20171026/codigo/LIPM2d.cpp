/*
 * 2dLIPM.cpp
 *
 *  Created on: 03/12/2015
 *      Author: teo
 */

#include "global.h"
#include "LIPM2d.h"

LIPM2d::LIPM2d()
{
    _A[0][0] = 0;
    _A[0][1] = 1;
    _A[1][0] = 6.9794;
    _A[1][1] = 0;
    _B[0][0] = 0;
    _B[1][0] = 1;
    _C[0] = -1.3061;
    _C[1] = 0;
    _D = 0.3257;
    _K[0] = 13.5366;
    _K[1] = 5.1035;
    _Ki = 10.0;
    _Kp = -0.5;
    //_Kd = -0.5;
    _Ku = 1.65;
    _T = 0.03;

    cout << "Discrete-time Space State Model description:" << endl;
    cout << "\n A = " << endl;
    cout << "\t x1 \t x2" << endl;
    cout << "x1 \t " << _A[0][0] << "\t " << _A[0][1] << endl;
    cout << "x2 \t " << _A[1][0] << "\t " << _A[1][1] << endl;
    cout << "\n B = " << endl;
    cout << "\t u" << endl;
    cout << "x1 \t " << _B[0][0] << endl;
    cout << "x2 \t " << _B[1][0] << endl;
    cout << "\n C = " << endl;
    cout << "\t x1 \t x2" << endl;
    cout << "y \t " << _C[0] << "\t " << _C[1] << endl;
    cout << "\n D = " << endl;
    cout << "\t u" << endl;
    cout << "y \t " << _D << endl;
    cout << "\nLinear Quadratic Regulator gain:" << endl;
    cout << "K = [ " << _K[0] << " , "<< _K[1] << " ]" << endl;
    cout << "\nSample Time : " << _T << " s" << endl;

    // Inicializacion variables
    _zmp_error = 0.0;
    _pre_zmp_error = 0.0;
    _u = 0.0;
    _x1[0] = 0.0;
    _x1[1] = 0.0*3.1415/180;
    _x2[0] = 0.0;
    _x2[1] = 0.0;
    _z[0] = 0.0;
    _z[1] = 0.0;
    _u_ref = 0.0;
    y = 0.0;
    pre_y = 0.0;
    dy = 0.0;
    _zmp_ref = 0.0; // ZMP reference

    PIDout = 0.0;
    Pout = 0.0;
    Iout = 0.0;
    Dout = 0.0;

}

LIPM2d::~LIPM2d(){
}

float LIPM2d::model(float zmp_real, float ref){
     /** STATE FEEDBACK WITH INTEGRAL ACTION **/

    // Control diagram where ""error = ZMPref - ZMPreal"" and y is not feedbacked, it is sent to the robot
/*
    _zmp_ref = ref;

    _x1[0] = _x1[1];
    _x2[0] = _x2[1];

    _u_ref = _zmp_ref/L; // L = 0.8927 is the pendulum longitude.
    _zmp_error = _zmp_ref - zmp_real;

    _u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Ki*(_pre_zmp_error + _zmp_error)*_T + _Kp*_zmp_error - _Ku*_u_ref;
    y = _C[0]*_x1[0] + _C[1]*_x2[0] + _D*_u;
//    dy = (y - pre_y) / _T; // velocity
*/

    _x1[0] = _x1[1];
    _x2[0] = _x2[1];
    _u_ref = (_zmp_ref/L)*pi/180;

    _x1[1] = _A[0][0]*_x1[0] + _A[0][1]*_x2[0] + _B[0][0]*_u_ref;
    _x2[1] = _A[1][0]*_x1[0] + _A[1][1]*_x2[0] + _B[1][0]*_u_ref;
    _x1[0] = _x1[1];
    _x2[0] = _x2[1];

    y = _C[0]*_x1[0] + _C[1]*_x2[0] + _D*_u;

    pre_y = y;
    _pre_zmp_error = _zmp_error;

    return y;

}
