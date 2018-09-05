#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"
#include "pid.h"
#include "global.h"

static FILE *fp;

yarp::os::Port port0;
yarp::os::Port port1;
yarp::os::Port portImu;

/** Left Leg Device */
yarp::dev::PolyDriver leftLegDevice;
/** Left Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *leftLegIControlMode2;
/** Left Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *leftLegIPositionControl2; // para control en posicion
/** Left Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *leftLegIVelocityControl2; // para control en velocidad

/** Right Leg Device */
yarp::dev::PolyDriver rightLegDevice;
/** Right Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *rightLegIControlMode2;
/** Right Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *rightLegIPositionControl2; // para control en posicion
/** Right Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *rightLegIVelocityControl2; // para control en velocidad

/** Trunk Device */
yarp::dev::PolyDriver trunkDevice;
/** Trunk ControlMode2 Interface */
yarp::dev::IControlMode2 *trunkIControlMode2;
/** Trunk PositionControl2 Interface */
yarp::dev::IPositionControl2 *trunkIPositionControl2; // para control en posicion
/** Trunk VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *trunkIVelocityControl2; // para control en velocidad

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    //Conversion to [ms]

        x_sensor.resize(samples); // tamaño del vector en funcion del numero de muestras para promediar con el filtro
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

        Xzmp_imu=0.0; // inicializacion del ZMP IMU
        Yzmp_imu=0.0;

        //Xzmp_b=0.0; // inicializacion del ZMP de Jaunlo
        //Yzmp_b=0.0;
        //Xzmp_off = 0.0; //inicializacion del ZMP con offset corregido de Juanlo

        seno_x = 0.0;
        seno_z = 0.0;
        coseno_z = 0.0;
        radianes = 0.0;
        ddx=0.0;
        ddy=0.0;
        ddz=9.8;
        X2 = 0.0;
        offs_x_l2 = 0.0;
        sum_l2 = 0.0;
        zmp_ref = 0.0;
        _angle_ref_1 = 0.0;
        _angle_ref = 0.0;
    }

    void run()
    {

            // Test escalon con rampa (fig. 9 y 12) // generacion del ZMP_ref para test FT sensor
            if (n <= 300){zmp_ref = 0.0;}
            else if (n >= 300 && n <= 330){zmp_ref = (0.05/30)*n - 0.5;} // variar desde 0.01 a 0.09
            else {zmp_ref = zmp_ref;}

            getInitialTime();
            
            readSensors(); // calculo del ZMP_FT
            zmpComp(); // calculo del ZMP_FT
            
            evalControl(); // Generacion de la actuacion en funcion del ZMP_ref
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

        Bottle ch0;
        Bottle ch1;
        Bottle imu;

        port0.read(ch0); // lectura del sensor JR3 ch0
        port1.read(ch1); // lectura del sensor JR3 ch1
        portImu.read(imu); // lectura del sensor IMU

        //--- FT-Sensor
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

        //--- Inertial-Sensor
        //ang_x = imu.get(0).asDouble(); // Angulo en X [deg]
        //ang_y = imu.get(1).asDouble(); // Angulo en Y [deg]
        //ang_z = imu.get(2).asDouble(); // Angulo en Z [deg]
        acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
        x_sensor.push_front(acc_x);
        x_sensor.pop_back();
        acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        y_sensor.push_front(acc_y);
        y_sensor.pop_back();
        acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
        z_sensor.push_front(acc_z);
        z_sensor.pop_back();
        //spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
        //spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
        //spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
        //mag_x=imu.get(9).asDouble(); // Campo magnetico en X
        //mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
        //mag_z=imu.get(11).asDouble(); // Campo magnetico en Z

        //LOW-PASS FILTER
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
        for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            ddx = ddx + *it;
        for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
            ddy = ddy + *it;
        for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
            ddz = ddz + *it;
        ddx = ddx / samples;
        ddy = ddy / samples;
        ddz = ddz / samples;

        //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
         ddx_robot = ddx;
         ddy_robot = -ddy;
         ddz_robot = ddz;

    }

    void zmpComp(){

        //ZMP Equations : Double Support - FT

/*      //Con el coeficiente 1000 - unidades en milimetros
        _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
        _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
        _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

        //Sin el coeficiente 1000 - unidades en metros
        _xzmp0_ft = -(((_my0/10) + e*_fx0)) / _fz0; // xzmp0_ft in [m]
        //_yzmp0_ft = (((_mx0/10) + e*_fy0)) / _fz0; // yzmp0_ft in [m]
        _xzmp1_ft = -(((_my1/10) + e*_fx1)) / _fz1; // xzmp1_ft in [m]
        //_yzmp1_ft = (((_mx1/10) + e*_fy1)) / _fz1; // yzmp1_ft in [m]

        _xzmp01_ft = (_xzmp0_ft * _fz0 + _xzmp1_ft * _fz1) / (_fz0 + _fz1); // xzmp_ft in [m]
        //_yzmp01_ft = (_yzmp0_ft * _fz0 + _yzmp1_ft * _fz1) / (_fz0 + _fz1); // yzmp_ft in [m]

        // OFFSET
        if (n >=1 && n < 50){
            sum_l = _xzmp01_ft + sum_l;
            offs_x_l = sum_l / n;
            printf("offs = %f\n", offs_x_l);
        }

        Xzmp_ft  = _xzmp01_ft - offs_x_l;
        //Yzmp_ft = _yzmp01_ft - offs_y;

        if ((_xzmp01_ft != _xzmp01_ft) || (_yzmp01_ft != _yzmp01_ft)){
            printf ("Warning: No zmp data\n");
        }


        //ZERO MOMENT POINT COMPUTATION - IMU
        Xzmp_imu = Xcom - (Zcom / ddz_robot) * ddx_robot; //ZMP X coordinate [m]
        //Yzmp_imu = Ycom - (Zcom / ddz_robot) * ddy_robot; //ZMP Y coordinate [m]

        ddz_CT = ddz_robot*cos(_angle_ref) + ddx_robot*sin(_angle_ref);
        ddx_CT = ddx_robot*cos(_angle_ref) - ddz_robot*sin(_angle_ref);
        newZcom = (Xzmp_ft * ddz_robot) / ddx_robot; // para evaluar cual es el valor mas adecuado para Zcom del modelo CT

    }

    void evalControl(){

/*    //-- Control Imu
    //DETERMINING STRATEGY
    lin_vel = x_sensor.at(0) * dt;
    w = sqrt(G / (Zcom / 100));
    capture_point = (lin_vel / w) + (Xzmp_imu / 100);

    //PID
    actual_value = Xzmp_imu;
    if (n==1){ setpoint = Xzmp_imu; } //Get initial position as setpoint [cm]
    //setpoint = 0;
    pid_output_ankle = pidcontroller_ankle->calculate(setpoint, actual_value);
    pid_output_hip = pidcontroller_hip->calculate(setpoint, actual_value);
*/

        //-- EVALUACION DLIPM MODELO
        _eval_Ctrl.model(Xzmp_ft,zmp_ref);

        ka = 0.25 * zmp_ref + 9.95;

        _angle_ref_1 = (zmp_ref*(-G))/ (L*(ka-G));
        _angle_ref =  -_eval_Ctrl.y + _angle_ref_1;

        ddz_CT = ddz_robot*cos(_angle_ref) + ddx_robot*sin(_angle_ref);
        ddx_CT = ddx_robot*cos(_angle_ref) - ddz_robot*sin(_angle_ref);
        newZcom = (Xzmp_ft * ddz_CT) / ddx_CT; // para evaluar cual es el valor mas adecuado para Zcom del modelo CT

    }

    void setJoints(){

        //-- ANKLE STRATEGY - POSITION CONTROL
        rightLegIPositionControl2->positionMove(4, _angle_ref); // position in degrees
        leftLegIPositionControl2->positionMove(4, _angle_ref);


/*        //-- HIP STRATEGY - POSITION CONTROL
        posRightLeg->positionMove(5, -angle_y); // axial ankle Right Leg
        posRightLeg->positionMove(1, -angle_y); // axial hip Right Leg
        posLeftLeg->positionMove(5, angle_y); // axial ankle Left Leg
        posLeftLeg->positionMove(1, angle_y); // axial hip Left Leg
        */

/*        //-- ANKLE STRATEGY - VELOCITY CONTROL
        velRightLeg->velocityMove(4, -vel); // velocity in degrees per second
        velLeftLeg->velocityMove(4, -vel);
*/
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
        cout << endl << "El angulo model es: " << _eval_Ctrl.y << endl;
        cout << endl << "El angulo total es: " << _angle_ref << endl;
        cout << endl << "El ka exp es: " << _eval_Ctrl._ka << endl;
        cout << endl << "El ka model es: " << _eval_Ctrl._ka_const << endl;
        cout << endl << "La ZMP ref es: " << zmp_ref << endl;
        cout << endl << "La ZMP FT es: " << Xzmp_ft << endl;
        cout << endl << "La ZMP IMU es: " << Xzmp_imu << endl;
        //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
        //cout << endl << "ZMP model es: " << _eval_x.y << endl;
        //cout << endl << "Num es: " << _num << endl;  _u_ref
        //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
        //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

    }

    void saveInFileCsv()
    {
        if(n==1){
            fprintf(fp,"Time,angle_ref,zmp_ref,zmp_ft,Xzmp_imu,ka,ba,newZcom,ddx_rob,ddz_rob,iter");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%.10f", _angle_ref);
        fprintf(fp,",%.10f", zmp_ref);
        fprintf(fp,",%.8f", Xzmp_ft);
        fprintf(fp,",%.8f", Xzmp_imu);
        fprintf(fp,",%.10f", _eval_Ctrl._ka);
        fprintf(fp,",%.10f", _eval_Ctrl._ba);
        fprintf(fp,",%.8f", newZcom);
        fprintf(fp,",%.10f", ddx_robot);
        fprintf(fp,",%.10f", ddz_robot);
        fprintf(fp,",%i", n);

    }

private:
    int m, n;
    double _num; // Variable para jugar con las iteraciones (500 iteraciones -> 15 segundos)
    float e; // distance [m] between ground and sensor center

    LIPM2d _eval_Ctrl;

    //-- IMU variables
    double acc_x, acc_y, acc_z, ang_x, ang_y, ang_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z; // IMU inputs
    //-- IMU LOW-FILTER variables & CONVERTION
    double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot, ddx_CT, ddz_CT; // additional acc variables

    double init_time, act_time, init_loop, act_loop, it_time, it_prev; // variables para los tiempos
    deque<double> x_sensor, y_sensor, z_sensor;

    //-- FT variables
    float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
    float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]
    //-- FT LOW-FILTER variables
    float offs_x_j, offs_x_l, offs_x_l2; // zmp offset in initial time.
    float offs_y;
    float sum_j, sum_l, sum_l2;

    //-- ZMP variables
    float _xzmp0_ft, _yzmp0_ft; // ZMP sensor 0 (derecho)
    float _xzmp1_ft, _yzmp1_ft; // ZMP sensor 1 (izquierdo)
    float _xzmp01_ft, _yzmp01_ft; // ZMP robot (zmp_0 + zmp_0) metros
    float Xzmp_ft, Yzmp_ft; // Global ZMP-FT despues de filtrar
    double Xzmp_imu, Yzmp_imu; // Global ZMP-IMU despues de filtrar
    //, Xzmp_b, Yzmp_b, Xzmp_c, Xzmp_off; // x, y, z en [cm]
    float newZcom; // para evaluar cual es el valor mas adecuado para Zcom del modelo CT

    //-- CONTROL/MOVEMENT variables
    float X, ref, angle_x, X2, zmp_ref, ref2, y2, _angle_ref, _angle_ref_1, _angle_ref_a, _angle_ref_b, _angle_ref_c, _angle_ref_d, ka;
    double _angulo, _angulo2, seno_x, seno_z, coseno_z, radianes; //Angulo para compensar las componentes de la aceleración en el modelo cart-table,
                              //Angulo2 para invertir los angulos negativos
    //-- SET & PID variables
    PID *pidcontroller_ankle, *pidcontroller_hip;
    std::string plane;
    double actual_value, setpoint, pid_output_ankle, pid_output_hip, initial_encoder;
    double capture_point, lin_vel, w;

};

#endif

