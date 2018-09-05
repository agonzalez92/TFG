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
    /*_A[0][0] = 1.003;
    _A[0][1] = 0.03003;
    _A[1][0] = 0.2096;
    _A[1][1] = 1.003;

    _B[0][0] = 0.0004503;
    _B[1][0] = 0.03003;

    _C[0] = -1.306;
    _C[1] = 0.0;

    _D = 0.3257;

    _K[0] = 13.5366;
    _K[1] = 5.1035;
    _Ki = 0.0; // Antes era 10.0
    _Kp = -0.0025; // Antes era -0.5
    _Kd = -0.00005;
    _Ku = 1.4; // antes era 1.65;
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
*/
    // Inicializacion variables
    _zmp_ref = 0.0;
    _ang_ref = 0.0;
    _zmp_ft = 0;

    _chi = 1;
    _a = 0.834;
    _b = 1.024;
    _c = -0.0004;

    _u = 0.0;
    y = 0.0;

    _x1[0] = 0.0;
    _x1[1] = 0.0;
    _x2[0] = 0.0;
    _x2[1] = 0.0;

    _z[0] = 0.0;
    _z[1] = 0.0;
    _u_ref = 0.0;

    pre_y = 0.0;
    dy = 0.0;

    PIDout = 0.0;
    Pout = 0.0;
    Iout = 0.0;
    Dout = 0.0;

    _Wn = 0.0;
    _ka = 9.91;
    _ka_const = 9.9575;
    _ba = 38.6844;
    //_ka = 9.9971; // para zmp_ref=0.09
//    _ba = 30.6949; // para chi=0.8 - para zmp_ref=0.09
    //_ba = 51.1582; // para chi = 1 - para zmp_ref=0.09

}


LIPM2d::~LIPM2d(){
}

float LIPM2d::model(float ft, float ref){
     /** STATE FEEDBACK WITH INTEGRAL ACTION **/

    //_zmp_ref = ref;

    _zmp_error =  ref - ft;
    //_zmp_error = _zmp_error + ref;

    _x1[0] = _x1[1];
    _x2[0] = _x2[1];

/*
    #define pi  3.14159265358979323846
    #define g   9.81  // Gravity in m/s²
    #define TS  0.03
    #define L   0.8927 // Pendulum Longitude [m]
    #define M   62.589 // Robot mass [kg]
*/
/*
    _chi = 1;

    % ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-g/L^2) / ((ka-g)./L)
    ANG_ref = -(asin(ZMP_ref/L) * 180/pi) ;
    ka = ((ANG_ref*-1*g/L)/ZMP_ref)+g;

    % Beta = (ka-g)./L = Wn^2
    Wn = sqrt((ka-g)/L);

    % Alpha = ba./(m*L) = 2*chi*Wn
    ba = 2*chi*Wn*m*L;

    K = -g/L^2;
    Num = K;

    Alpha = ba/(m*L);
    Beta = (ka-g)/L;
*/

    _ka_const = 0.25 * ref + 9.95;
    _ang_ref = (ref*(-g))/ (L*(_ka_const-g)) ;

    //_ang_ref = -(asin(ref/L)*180/pi); // relacion entre zmp_ref y ang_ref

    _zmp_ft = _a*ref*ref + _b*ref + _c; // sacado del exp1
    if (_ang_ref==0 || _zmp_ft<0.00000000001)    {
        _ka = 9.95;
    }
    else    {
        _ka = ((_zmp_ft*(-1)*g/L)/_ang_ref)+g; // calculo de ka en reg. estacionario
    }

    _Wn = sqrt((_ka-g))/L; // basado en la forma general de una ft de 2ºorden
    _ba = 2*_chi*_Wn*M*L; // aplicando el proceso inverso de comparacion de la linea anterior

    _A[0][0] = 0.0;
    _A[0][1] = 1.0;
    _A[1][0] = -((_ka-g)/L);
    _A[1][1] = -(_ba/(M*L));

    _B[0][0] = 0.0;
    _B[1][0] = 1.0;

    _C[0] = -g/(L*L);
    _C[1] = 0.0;

    _D = 0.0;

    /*
    //_u_ref = _zmp_ref/L; // L = 0.8927 is the pendulum longitude.
    //_u_ref = (_zmp_ref/L) * 180 / pi;

    //_zmp_error = _zmp_ref - zmp_real;
    //_angle_error = (_zmp_error/L) * 180 / pi;

    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Ki*(_pre_zmp_error + _zmp_error)*_T + _Kp*_zmp_error - _Ku*_u_ref; // basico - LOLI
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Ki*(_pre_zmp_error + _zmp_error)*_T + _Kp*_zmp_error - _Ku*_u_ref; //
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Kp*_zmp_error - _Ku*_u_ref; // sin Ki
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Kp*_zmp_error + _Kd*((_zmp_error-_pre_zmp_error)/_T) - _Ku*_u_ref; // sin Ki
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + (_pre_zmp_error + _zmp_error) - _Ku*_u_ref;
    //_zmp_PD = _Kp*_zmp_error + _Kd*((_zmp_error-_pre_zmp_error)/_T);
    //_angle_error =  (_zmp_PD/L) * 180 / pi;; // sin Ki
    //_u =  _angle_error -_K[0]*_x1[0] -_K[1]*_x2[0] - _Ku*_u_ref;
*/

    _u = _zmp_error; //bucle abierto. nuestra entrada es directamente la refencia de ZMP que queramos.
    y = _C[0]*_x1[0] + _C[1]*_x2[0] + _D*_u;
//    dy = (y - pre_y) / _T; // velocity

    _x1[1] = _A[0][0]*_x1[0] + _A[0][1]*_x2[0] + _B[0][0]*_u;
    _x2[1] = _A[1][0]*_x1[0] + _A[1][1]*_x2[0] + _B[1][0]*_u;

    //pre_y = y;
    //_pre_zmp_error = _zmp_error;

    return y;

}
