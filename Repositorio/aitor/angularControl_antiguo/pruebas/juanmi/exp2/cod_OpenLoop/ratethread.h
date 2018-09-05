#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"

static FILE *fp;

yarp::os::Port port0;
yarp::os::Port port1;
yarp::os::Port portImu;
yarp::dev::IPositionControl2 *posRightLeg;
yarp::dev::IPositionControl2 *posLeftLeg;


class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(TS*1000.0) {    //Conversion to [ms]
        x_sensor.resize(samples);
        y_sensor.resize(samples);
        z_sensor.resize(samples);
        n = 1;
        m = 0;
        e = 0.03225;  // Loli 0.0194
        sum_j = 0.0;
        sum_l = 0.0;
        offs_x_j = 0.0;
        offs_x_l = 0.0;
        offs_y = 0.0;
        X = 0.0;
        _angulo = 0.0; // El angulo es 0 para inclinar el robot hasta -10 grados
        _num = 1;
        _angulo2 = 0.0;
        Xzmp=0.0; // inicializacion del ZMP de Jaunlo
        Yzmp=0.0;
        Xzmp_b=0.0; // inicializacion del ZMP de Jaunlo
        Yzmp_b=0.0;
        Xzmp_off = 0.0; //inicializacion del ZMP con offset corregido de Juanlo
        seno_x = 0.0;
        seno_z = 0.0;
        coseno_z = 0.0;
        radianes = 0.0;
        x=0.0;
        z=9.8;
        X2 = 0.0;
        offs_x_l2 = 0.0;
        sum_l2 = 0.0;
        zmp_ref = 0.0;
        _angle_ref_1 = 0.0;
        _angle_ref = 0.0;
    }

    void run()
    {

            // Test escalon con rampa (fig. 9 y 12) // generacion del ZMP_ref
            if (n <= 300){zmp_ref = 0.0;}
            else if (n >= 300 && n <= 330){zmp_ref = (0.1/30)*n - 1;} // variar desde 0.01 a 0.09
            else {zmp_ref = zmp_ref;}

            getInitialTime();
            
            readSensors(); // calculo del ZMP_FT
            zmpComp2(); // calculo del ZMP_FT
            
            evaluateModel(); // Generacion de la actuacion en funcion del ZMP_ref
            setJoints(); // Generacion de la actuacion en funcion del ZMP_ref

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();
            n++;
            cout << n << endl << endl;
            getCurrentTime();

    }

    void getInitialTime()
    {
        if (n==1){init_time = Time::now();}
        init_loop = Time::now();
        it_time = init_loop - it_prev;
        it_prev = init_loop;
    }

    void getCurrentTime()
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
    }

    void readSensors(){

        /** FT-Sensor **/
        Bottle ch0;
        Bottle ch1;
        // Bottle imu;

        port0.read(ch0); // lectura del sensor JR3 ch0
        port1.read(ch1); // lectura del sensor JR3 ch1
        // portImu.read(imu); // lectura del sensor IMU

        _fx0 = ch0.get(0).asDouble();
        _fy0 = ch0.get(1).asDouble();
        _fz0 = ch0.get(2).asDouble();
        _mx0 = ch0.get(3).asDouble();
        _my0 = ch0.get(4).asDouble();
        _mz0 = ch0.get(5).asDouble();

        _fx1 = ch1.get(0).asDouble();
        _fy1 = ch1.get(1).asDouble();
        _fz1 = ch1.get(2).asDouble();
        _mx1 = ch1.get(3).asDouble();
        _my1 = ch1.get(4).asDouble();
        _mz1 = ch1.get(5).asDouble();

/*        // Inertial-Sensor
        ang_x = imu.get(0).asDouble(); // Angulo en X [deg]
        ang_y = imu.get(1).asDouble(); // Angulo en Y [deg]
        ang_z = imu.get(2).asDouble(); // Angulo en Z [deg]
        acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
        x_sensor.push_front(acc_x);
        x_sensor.pop_back();
        acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        y_sensor.push_front(acc_y);
        y_sensor.pop_back();
        acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
        z_sensor.push_front(acc_z);
        z_sensor.pop_back();
        spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
        spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
        spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
        mag_x=imu.get(9).asDouble(); // Campo magnetico en X
        mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
        mag_z=imu.get(11).asDouble(); // Campo magnetico en Z
        */

    }

    void zmpComp2(){

        //ZMP Equations : Double Support - LOLI

         //Con el coeficiente 1000
        //_xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
        //_yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

        //_xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
        //_yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]


        //Sin el coeficiente 1000
        _xzmp0 = -(((_my0/10) + e*_fx0)) / _fz0; // xzmp0 in [m]
        _yzmp0 = (((_mx0/10) + e*_fy0)) / _fz0; // yzmp0 in [m]

        _xzmp1 = -(((_my1/10) + e*_fx1)) / _fz1; // xzmp1 in [m]
        _yzmp1 = (((_mx1/10) + e*_fy1)) / _fz1; // yzmp1 in [m]

        _xzmp = (_xzmp0 * _fz0 + _xzmp1 * _fz1) / (_fz0 + _fz1); // xzmp in [m]
        _yzmp = (_yzmp0 * _fz0 + _yzmp1 * _fz1) / (_fz0 + _fz1); // yzmp in [m]

        // OFFSET
        if (n >=1 && n < 50){
            sum_l = _xzmp + sum_l;
            offs_x_l = sum_l / n;
            printf("offs = %f\n", offs_x_l);
        }

        X  = _xzmp - offs_x_l;
        //X = floor(10000*X+0.5)/10000;
        _yzmp = _yzmp - offs_y;

        if ((_xzmp != _xzmp) || (_yzmp != _yzmp)){
            printf ("Warning: No zmp data\n");
        }


        //_xzmp01 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp01 in [mm]
        //_yzmp01 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp01 in [mm]

        //_xzmp11 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp11 in [mm]
        //_yzmp11 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp11 in [mm]

        //_xzmp2 = (_xzmp01 * _fz0 + _xzmp11 * _fz1) / (_fz0 + _fz1); // xzmp2 in [mm]
        //_yzmp2 = (_yzmp01 * _fz0 + _yzmp11 * _fz1) / (_fz0 + _fz1); // yzmp2 in [mm]


        _xzmp01 = -(_my0 + e*_fx0) / _fz0; // xzmp01 in [mm]
        _yzmp01 = (_mx0 + e*_fy0) / _fz0; // yzmp01 in [mm]

        _xzmp11 = -(_my1 + e*_fx1) / _fz1; // xzmp11 in [mm]
        _yzmp11 = (_mx1 + e*_fy1) / _fz1; // yzmp11 in [mm]

        _xzmp2 = (_xzmp01 * _fz0 + _xzmp11 * _fz1) / (_fz0 + _fz1); // xzmp2 in [mm]
        _yzmp2 = (_yzmp01 * _fz0 + _yzmp11 * _fz1) / (_fz0 + _fz1); // yzmp2 in [mm]


        // OFFSET
        if (n >=1 && n < 50){
            sum_l2 = _xzmp2 + sum_l2;
            offs_x_l2 = sum_l2 / n;
            printf("offs = %f\n", offs_x_l);
        }

        X2 = _xzmp2 - offs_x_l2;
    }

    void evaluateModel(){
        /** EVALUACION MODELO **/
        _eval_x.model(X,zmp_ref);

        ka = 0.25 * zmp_ref + 9.95;
        //_angle_ref_1 =  - (asin(zmp_ref/L)*180/pi);

        _angle_ref_a = zmp_ref*(-g) ;
        _angle_ref_b = (ka-g);
        _angle_ref_c = L;
        _angle_ref_d = L*_angle_ref_b;
        _angle_ref_1 = _angle_ref_a/_angle_ref_d;
        //_angle_ref_1 = (zmp_ref*(-g))/ (L*(ka-g));
        _angle_ref =  -_eval_x.y + _angle_ref_1;

        /*
        //_angle_ref = floor(1000*_angle_ref+0.5)/1000;

        //angle_x = -(_eval_x.y-(4.3948*pow(_eval_x.y,2))+0.23*_eval_x.y)/0.0135;
        //y2 = _eval_x.y * 1000;
        //angle_x = - 0.05739*y2 - 0.07;
        //ref2 = ref;
        //angle_x = - 0.05739*(ref2) - 0.07;

//      angle_y = asin(_eval_x.y/1.03)*180/PI;
//      vel = 0.35* _eval_x.dy * (1/L) * (180/PI); //velocity in degrees per second
        */
    }

    void setJoints(){
        /** Position control **/
        //posRightLeg->positionMove(4, angle_x); // position in degrees
        //posLeftLeg->positionMove(4, angle_x);
        posRightLeg->positionMove(4, _angle_ref); // position in degrees
        posLeftLeg->positionMove(4, _angle_ref);
        //        posRightLeg->positionMove(5, -angle_y); // axial ankle Right Leg
        //        posRightLeg->positionMove(1, -angle_y); // axial hip Right Leg
        //        posLeftLeg->positionMove(5, angle_y); // axial ankle Left Leg
        //        posLeftLeg->positionMove(1, angle_y); // axial hip Left Leg

        /** Velocity control **/
//        velRightLeg->velocityMove(4, -vel); // velocity in degrees per second
//        velLeftLeg->velocityMove(4, -vel);
    }

    void printData()
    {
/*        cout << endl << "El angulo 1 es: " << _angle_ref_a << endl;
        cout << endl << "El angulo 2 es: " << _angle_ref_b << endl;
        cout << endl << "El angulo 3 es: " << _angle_ref_c << endl;
        cout << endl << "El angulo 4 es: " << _angle_ref_d << endl;
        cout << endl << "El angulo 4 es: " << g << endl;
        cout << endl << "El angulo 4 es: " << ka << endl;
*/
        cout << endl << "El angulo ref es: " << _angle_ref_1 << endl;
        cout << endl << "El angulo model es: " << _eval_x.y << endl;
        cout << endl << "El angulo total es: " << _angle_ref << endl;
        cout << endl << "El ka exp es: " << _eval_x._ka << endl;
        cout << endl << "El ka model es: " << _eval_x._ka_const << endl;
        cout << endl << "El ZMP FT es: " << X << endl;
        cout << endl << "La ZMP ref es: " << zmp_ref << endl;
        //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
        //cout << endl << "ZMP model es: " << _eval_x.y << endl;
        //cout << endl << "Num es: " << _num << endl;  _u_ref
        //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
        //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

    }

    void saveInFileCsv()
    {
        if(n==1){
            fprintf(fp,"Time,angle_ref,zmp_ref,zmp_ft,ka,ba,iter");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%.10f", _angle_ref);
        fprintf(fp,",%.10f", zmp_ref);
        fprintf(fp,",%.8f", X);
        fprintf(fp,",%.10f", _eval_x._ka);
        fprintf(fp,",%.10f", _eval_x._ba);
        fprintf(fp,",%i", n);

    }


private:
    int m, n;
    LIPM2d _eval_x;
    double acc_x, acc_y, acc_z, x, y, z, x_acc_robot, y_acc_robot, z_acc_robot, Xzmp, Yzmp, Xzmp_b, Yzmp_b, Xzmp_c, Xzmp_off; // x, y, z en [cm]
    double ang_x, ang_y, ang_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z;
    double init_time, act_time, init_loop, act_loop, it_time, it_prev;
    deque<double> x_sensor, y_sensor, z_sensor;
    double _num; // Variable para jugar con las iteraciones (500 iteraciones -> 15 segundos)

    float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
    float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]

    float e; // distance [m] between ground and sensor center
    float offs_x_j, offs_x_l, offs_x_l2; // zmp offset in initial time.
    float offs_y;
    float sum_j, sum_l, sum_l2;
    float _xzmp0, _yzmp0; // ZMP sensor 0 (derecho)
    float _xzmp1, _yzmp1; // ZMP sensor 1 (izquierdo)
    float _xzmp01, _xzmp11, _xzmp2, _yzmp01, _yzmp11, _yzmp2; // variables para graficar en milimetros
    float _xzmp; // Global x_ZMP
    float _yzmp; // Global y_ZMP
    float X, ref, angle_x, X2, zmp_ref, ref2, y2, _angle_ref, _angle_ref_1, _angle_ref_a, _angle_ref_b, _angle_ref_c, _angle_ref_d, ka;
    double _angulo, _angulo2, seno_x, seno_z, coseno_z, radianes; //Angulo para compensar las componentes de la aceleración en el modelo cart-table,
                              //Angulo2 para invertir los angulos negativos

};

#endif

