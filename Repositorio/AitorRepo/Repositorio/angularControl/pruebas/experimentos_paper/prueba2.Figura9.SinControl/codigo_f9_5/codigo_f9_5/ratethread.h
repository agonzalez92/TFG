#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"

static FILE *fp;

yarp::os::Port port0;
yarp::os::Port port1;
yarp::os::Port portImu;
yarp::dev::IPositionControl *posRightLeg;
yarp::dev::IPositionControl *posLeftLeg;


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
    }

    void run()
    {

        if (m!=300){
           mediumIMU();
        }

        if (m>=300){

            // Test escalon con rampa (fig. 9 y 12)
            if (n <= 300){ref = 0.0;}
            else if (n >= 300 && n <= 330){ref = (0.05/30)*n - 0.5;}
            else {ref = ref;}

            getInitialTime();
            readSensors();
            //angular_movement();
            //zmpref_generator();
            //zmpComp(_angulo2);
            angleComp();
            zmpComp2();
            //evaluateModel();
            setJoints();

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileTxt();
            saveInFileCsv();
            n++;
            cout << n << endl << endl;
            getCurrentTime();
        }
    }

    void angular_movement(){

        if(n==1){
            /** POSITION CONTROL. RIGHT AND LEFT LEGS**/
            cout << "test Right Leg positionMove(4," << _angulo  << ")" << endl;
            posRightLeg->positionMove(4, _angulo);
            cout << "test Left Leg positionMove(4," << _angulo  << ")" << endl;
            posLeftLeg->positionMove(4, _angulo);
        }

        if(_num<500){
            _num++;
            //printf("\n %f",_num);
            return;
        }
        else{
            _num = 1;
            _angulo = _angulo - 0.5;
            _angulo2 = _angulo * (-1);
            /** POSITION CONTROL. RIGHT AND LEFT LEGS**/
            cout << "test Right Leg positionMove(4," << _angulo  << ")" << endl;
            posRightLeg->positionMove(4, _angulo);
            cout << "test Left Leg positionMove(4," << _angulo  << ")" << endl;
            posLeftLeg->positionMove(4, _angulo);
        }

        return;

    }

    void zmpref_generator(){
        if(zmp_ref < 0.1){
            if (_num < 330){
                _num++;
                return;
            }else{
                _num = 1;
                ref = zmp_ref + 0.005;
                zmp_ref=ref;
                return;
            }
        }

        return;
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
        Bottle imu;

        port0.read(ch0); // lectura del sensor JR3 ch0
        port1.read(ch1); // lectura del sensor JR3 ch1
        portImu.read(imu); // lectura del sensor IMU

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

        /** Inertial-Sensor **/

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
    }

    void mediumIMU(){
        Bottle imu;
        portImu.read(imu); // lectura del sensor IMU

        acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
        x_sensor.push_front(acc_x);
        x_sensor.pop_back();
        acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        y_sensor.push_front(acc_y);
        y_sensor.pop_back();
        acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
        z_sensor.push_front(acc_z);
        z_sensor.pop_back();

        for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            x = x + *it;
        for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
            y = y + *it;
        for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
            z = z + *it;

        x = x / samples;
        y = y / samples;
        z = z / samples;

        m++;
    }

    void zmpComp(double angulo){

        /** JUANLO **/
        //LOW-PASS FILTER

        for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            x = x + *it;
        for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
            y = y + *it;
        for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
            z = z + *it;

        x = x / samples;
        y = y / samples;
        z = z / samples;

    //-------------------------------------------------------------------------------


        //CONVERSION FROM SENSOR COORDINATES TO ROBOT COORDINATES
        x_acc_robot = x; //para filtro
        y_acc_robot = -y;
        z_acc_robot = z;

        radianes = (angulo*pi)/180;

        seno_x = sin(radianes);
        seno_z = sin(radianes);
        coseno_z = cos(radianes);



        //ZERO MOMENT POINT COMPUTATION
        //Xzmp -
        Xzmp =  - (((Zcom*10)  * (x_acc_robot*seno_x - z_acc_robot*coseno_z)) / (g - z_acc_robot*seno_z)); //ZMP X coordinate [mm], angulo [rad]
        Yzmp = Yzmp - ((Zcom*10) * y_acc_robot) / (z_acc_robot*seno_z); //ZMP Y coordinate [mm]

        //Xzmp = Xzmp_c - Xzmp_b;// - offs_x_j;
        //Xzmp_b = Xzmp;

        //Yzmp = Yzmp - Yzmp_b;
        //Yzmp_b = Yzmp;

        // OFFSET

        if (n >=1 && n < 50){
            sum_j = Xzmp + sum_j;
            offs_x_j = sum_j / n;
            printf("offs = %f\n", offs_x_j);
        }

        Xzmp = Xzmp - offs_x_j;

        /** LOLI **/
        //ZMP Equations : Double Support


        _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
        _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
        _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]


        _xzmp = (_xzmp0 * _fz0 + _xzmp1 * _fz1) / (_fz0 + _fz1); // xzmp in [mm]
        _yzmp = (_yzmp0 * _fz0 + _yzmp1 * _fz1) / (_fz0 + _fz1); // yzmp in [mm]


        // OFFSET
        if (n >=1 && n < 50){
            sum_l = _xzmp + sum_l;
            offs_x_l = sum_l / n;
            printf("offs = %f\n", offs_x_l);
        }

        X  = _xzmp - offs_x_l;
        _yzmp = _yzmp - offs_y;

        if ((_xzmp != _xzmp) || (_yzmp != _yzmp)){
            printf ("Warning: No zmp data\n");
        }

    }

    void angleComp(){
        //_angle_ref = -(90 -((acos (ref/L)) * 180.0 / pi));
        //_angle_ref = (acos ((e - ref)/L) * 180.0 / pi);
        _angle_ref = -(_eval_x.y-(4.3948*pow(_eval_x.y,2))+0.23*_eval_x.y)/0.0135;
        return;
    }

    void zmpComp2(){
        /** LOLI **/
        //ZMP Equations : Double Support


/*         //Con el coeficiente 1000
        _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
        _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
        _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

        //Sin el coeficiente 1000
        _xzmp0 = -(((_my0/10) + e*_fx0)) / _fz0; // xzmp0 in [mm]
        _yzmp0 = (((_mx0/10) + e*_fy0)) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -(((_my1/10) + e*_fx1)) / _fz1; // xzmp1 in [mm]
        _yzmp1 = (((_mx1/10) + e*_fy1)) / _fz1; // yzmp1 in [mm]

        _xzmp = (_xzmp0 * _fz0 + _xzmp1 * _fz1) / (_fz0 + _fz1); // xzmp in [mm]
        _yzmp = (_yzmp0 * _fz0 + _yzmp1 * _fz1) / (_fz0 + _fz1); // yzmp in [mm]

        // OFFSET
        if (n >=1 && n < 50){
            sum_l = _xzmp + sum_l;
            offs_x_l = sum_l / n;
            printf("offs = %f\n", offs_x_l);
        }

        X  = _xzmp - offs_x_l;
        _yzmp = _yzmp - offs_y;

        if ((_xzmp != _xzmp) || (_yzmp != _yzmp)){
            printf ("Warning: No zmp data\n");
        }


        /*
        _xzmp01 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp01 in [mm]
        _yzmp01 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp01 in [mm]

        _xzmp11 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp11 in [mm]
        _yzmp11 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp11 in [mm]

        _xzmp2 = (_xzmp01 * _fz0 + _xzmp11 * _fz1) / (_fz0 + _fz1); // xzmp2 in [mm]
        _yzmp2 = (_yzmp01 * _fz0 + _yzmp11 * _fz1) / (_fz0 + _fz1); // yzmp2 in [mm]
        */

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
        _eval_x.model(X, ref);
        // _eval_y.model(_yzmp);

        angle_x = -(_eval_x.y-(4.3948*pow(_eval_x.y,2))+0.23*_eval_x.y)/0.0135;
        //y2 = _eval_x.y * 1000;
        //angle_x = - 0.05739*y2 - 0.07;
        //ref2 = ref;
        //angle_x = - 0.05739*(ref2) - 0.07;

//      angle_y = asin(_eval_x.y/1.03)*180/PI;
//      vel = 0.35* _eval_x.dy * (1/L) * (180/PI); //velocity in degrees per second
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

        //cout << endl << "El angulo es: " << angle_x << endl;
        cout << endl << "El angulo es: " << _angle_ref << endl;
        cout << endl << "La ref es: " << ref << endl;
        cout << endl << "ZMP_Error_Loli = ("<<_eval_x._zmp_error << ") [mm]" << endl;
        cout << endl << "El angulo2 es: " << _eval_x.y << endl;
        //cout << endl << "Num es: " << _num << endl;

    }

    void saveInFileTxt()
    {
        ofstream out;
        out.open("data_ZMP.txt",ios::app);

        out << "Time: " << act_time << " ";
        out << "Angulo: " << _angulo2 << " ";
        out << "Xcom_Juanlo: " << Xcom << " ";
        out << "Ycom_Juanlo: " << Ycom << " ";
        out << "Zcom_Juanlo: " << Zcom << " ";
        out << "x_acc_robot_Juanlo: " << x_acc_robot << " ";
        out << "y_acc_robot_Juanlo: " << y_acc_robot << " ";
        out << "z_acc_robot_Juanlo: " << z_acc_robot << " ";
        out << "Angle_X_Juanlo: " << ang_x << " ";
        out << "Angle_Y_Juanlo: " << ang_y << " ";
        out << "Angle_Z_Juanlo: " << ang_z << " ";
        out << "Acceleration_X_Juanlo: " << acc_x << " ";
        out << "Acceleration_Y_Juanlo: " << acc_y << " ";
        out << "Acceleration_Z_Juanlo: " << acc_z << " ";
        out << "Angular_speed_X_Juanlo: " << spd_x << " ";
        out << "Angular_speed_Y_Juanlo: " << spd_y << " ";
        out << "Angular_speed_Z_Juanlo: " << spd_z << " ";
        out << "Magnetic_field_X_Juanlo: " << mag_x << " ";
        out << "Magnetic_field_Y_Juanlo: " << mag_y << " ";
        out << "Magnetic_field_Z_Juanlo: " << mag_z << " ";
        out << "Seno en Z: " << seno_x << " ";
        out << "Coseno en X: " << seno_z << " ";
        out << "Coseno en Z: " << coseno_z << " ";
        out << "Xzmp_Juanlo: " << Xzmp << " ";
        out << "Yzmp_Juanlo: " << Yzmp << " ";
        out << "fx0_Loli: " << _fx0 << " ";
        out << "fy0_Loli: " << _fy0 << " ";
        out << "fz0_Loli: " << _fz0 << " ";
        out << "mx0_Loli: " << _mx0 << " ";
        out << "my0_Loli: " << _my0 << " ";
        out << "mz0_Loli: " << _mz0 << " ";
        out << "xzmp0_Loli: " << _xzmp0 << " ";
        out << "yzmp0_Loli: " << _yzmp0 << " ";
        out << "fx1_Loli: " << _fx1 << " ";
        out << "fy1_Loli: " << _fy1 << " ";
        out << "fz1_Loli: " << _fz1 << " ";
        out << "mx1_Loli: " << _mx1 << " ";
        out << "my1_Loli: " << _my1 << " ";
        out << "mz1_Loli: " << _mz1 << " ";
        out << "xzmp1_Loli: " << _xzmp1 << " ";
        out << "yzmp1_Loli: " << _yzmp1 << " ";
        out << "Xzmp_Loli: " << X << " ";
        out << "Yzmp_Loli: " << _yzmp << " ";
        out << endl;

        out.close();
    }

    void saveInFileCsv()
    {
        if(n==1){
            fprintf(fp,"Tiempo,Angulo,Xcom,Ycom,Zcom,x_acc_robot,y_acc_robot,z_acc_robot,angulo_x,angulo_y,angulo_z,aceleracion_x,aceleracion_y,aceleracion_z");
            fprintf(fp,",velocidad_angular_x,velocidad_angular_y,velocidad_angular_z,campo_magnetico_x,campo_magnetico_y,campo_magnetico_z");
            fprintf(fp,",seno_x,seno_z,coseno_z,Xzmp-Juanlo,Yzmp-Juanlo");
            fprintf(fp,",Xzmp_b,Xzmp_c,Xzmp_off,Yzmp_b,fx0,fy0,fz0,mx0,my0,mz0,Xzmp0,Yzmp0,fx1,fy1,fz1,mx1,my1,mz1,Xzmp1,Yzmp1,Xzmp-Loli,Yzmp-Loli");
            fprintf(fp,",evalx.y,eval_x.zmp_error,eval_x.zmp_ref,eval_x._u,eval_x._x1[0],eval_x._x2[0],angle_x,xzmp01,yzmp01,xzmp11,yzmp11,xzmp2,yzmp2,XzmpLoli2");
            fprintf(fp,",ZMPref_mm,eval_x.zmp_ref_mm,evalx.y_mm,angle_ref,zmp_ref");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%.2f",_angulo2);
        fprintf(fp,",%i", Xcom);
        fprintf(fp,",%i", Ycom);
        fprintf(fp,",%.2f", Zcom);
        fprintf(fp,",%.2f", x_acc_robot);
        fprintf(fp,",%.2f", y_acc_robot);
        fprintf(fp,",%.2f", z_acc_robot);
        fprintf(fp,",%.2f", ang_x);
        fprintf(fp,",%.2f", ang_y);
        fprintf(fp,",%.2f", ang_z);
        fprintf(fp,",%.2f", acc_x);
        fprintf(fp,",%.2f", acc_y);
        fprintf(fp,",%.2f", acc_z);
        fprintf(fp,",%.2f", spd_x);
        fprintf(fp,",%.2f", spd_y);
        fprintf(fp,",%.2f", spd_z);
        fprintf(fp,",%.2f", mag_x);
        fprintf(fp,",%.2f", mag_y);
        fprintf(fp,",%.2f", mag_z);
        fprintf(fp,",%.2f", seno_x);
        fprintf(fp,",%.2f", seno_z);
        fprintf(fp,",%.2f", coseno_z);
        fprintf(fp,",%.2f", Xzmp);
        fprintf(fp,",%.2f", Yzmp);
        fprintf(fp,",%.2f", Xzmp_b);
        fprintf(fp,",%.2f", Xzmp_c);
        fprintf(fp,",%.2f", Xzmp_off);
        fprintf(fp,",%.2f", Yzmp_b);
        fprintf(fp,",%.8f", _fx0);
        fprintf(fp,",%.8f", _fy0);
        fprintf(fp,",%.8f", _fz0);
        fprintf(fp,",%.8f", _mx0);
        fprintf(fp,",%.8f", _my0);
        fprintf(fp,",%.8f", _mz0);
        fprintf(fp,",%.8f", _xzmp0);
        fprintf(fp,",%.8f", _yzmp0);
        fprintf(fp,",%.8f", _fx1);
        fprintf(fp,",%.8f", _fy1);
        fprintf(fp,",%.8f", _fz1);
        fprintf(fp,",%.8f", _mx1);
        fprintf(fp,",%.8f", _my1);
        fprintf(fp,",%.8f", _mz1);
        fprintf(fp,",%.8f", _xzmp1);
        fprintf(fp,",%.8f", _yzmp1);
        fprintf(fp,",%.8f", X);
        fprintf(fp,",%.8f", _yzmp);
        fprintf(fp,",%.15f", _eval_x.y);
        fprintf(fp,",%.15f", _eval_x._zmp_error);
        fprintf(fp,",%.15f", _eval_x._zmp_ref);
        fprintf(fp,",%.10f", _eval_x._u);
        fprintf(fp,",%.10f", _eval_x._x1[0]);
        fprintf(fp,",%.10f", _eval_x._x2[0]);
        fprintf(fp,",%.2f", angle_x);
        fprintf(fp,",%.2f", _xzmp01);
        fprintf(fp,",%.2f", _yzmp01);
        fprintf(fp,",%.2f", _xzmp11);
        fprintf(fp,",%.2f", _yzmp11);
        fprintf(fp,",%.2f", _xzmp2);
        fprintf(fp,",%.2f", _yzmp2);
        fprintf(fp,",%.2f", X2);
        fprintf(fp,",%.15f", zmp_ref);
        fprintf(fp,",%.10f", ref2);
        fprintf(fp,",%.10f", y2);
        fprintf(fp,",%.10f", _angle_ref);
        fprintf(fp,",%.10f", ref);
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
    float X, ref, angle_x, X2, zmp_ref, ref2, y2, _angle_ref;
    double _angulo, _angulo2, seno_x, seno_z, coseno_z, radianes; //Angulo para compensar las componentes de la aceleración en el modelo cart-table,
                              //Angulo2 para invertir los angulos negativos

};

#endif
