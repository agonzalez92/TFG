#ifndef _ratethread_H_
#define _ratethread_H_

static FILE *fp;

yarp::os::Port port0;
yarp::os::Port port1;
yarp::os::Port portImu;

using namespace yarp::os;

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(TS*1000.0) {    //Conversion to [ms]
        x_sensor.resize(samples);
        y_sensor.resize(samples);
        z_sensor.resize(samples);
        n = 1;
        e = 0.03225;  // Loli 0.0194
        sum_j = 0.0;
        sum_l = 0.0;
        offs_x_j = 0.0;
        offs_x_l = 0.0;
        offs_y = 0.0;
        X = 0.0;
    }


    void run()
    {
        getInitialTime();
        readSensors();
        zmpComp();
        printData();
        cout << endl << "Press ENTER to exit..." << endl;
        cout << "*******************************" << endl << endl;
        saveInFileTxt();
        saveInFileCsv();
        n++;
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

    void zmpComp(){

        /** JUANLO **/
        //LOW-PASS FILTER
        x = 0.0;
        y = 0.0;
        z = 0.0;
        for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            x = x + *it;
        for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
            y = y + *it;
        for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
            z = z + *it;
        x = x / samples;
        y = y / samples;
        z = z / samples;

        //CONVERSION FROM SENSOR COORDINATES TO ROBOT COORDINATES
        x_robot = x;
        y_robot = -y;
        z_robot = z;

        //ZERO MOMENT POINT COMPUTATION
        Xzmp = (Xcom - (Zcom / z_robot) * x_robot)*10; //ZMP X coordinate [mm]
        Yzmp = (Ycom - (Zcom / z_robot) * y_robot)*10; //ZMP Y coordinate [mm]

        // OFFSET
        if (n >=1 && n < 50){
            sum_j = Xzmp + sum_j;
            offs_x_j = sum_j / n;
            printf("offs = %f\n", offs_x_j);
        }

        Xzmp = Xzmp - offs_x_j;

        /** LOLI **/
        //ZMP Equations : Double Support
        /*_xzmp0 = -((_my0*10000) + (e*_fx0)*100) / _fz0; // xzmp0 in [mm]
        _yzmp0 = ((_mx0*10000) + (e*_fy0)*100) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -((_my1*10000) + (e*_fx1)*100) / _fz1; // xzmp1 in [mm]
        _yzmp1 = ((_mx1*10000) + (e*_fy1)*100) / _fz1; // yzmp1 in [mm]*/

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


    void printData()
    {
        cout << "Angle X: " << ang_x << " deg" << endl;
        cout << "Angle Y: " << ang_y << " deg" << endl;
        cout << "Angle Z: " << ang_z << " deg" << endl << endl;
        cout << "Acceleration in X: " << acc_x << " m/s²" << endl;
        cout << "Acceleration in Y: " << acc_y << " m/s²" << endl;
        cout << "Acceleration in Z: " << acc_z << " m/s²" << endl << endl;
        cout << "Angular speed X: " << spd_x << " deg/s" << endl;
        cout << "Angular speed Y: " << spd_y << " deg/s" << endl;
        cout << "Angular speed Z: " << spd_z << " deg/s" << endl << endl;
        cout << "Magnetic field X: " << mag_x << endl;
        cout << "Magnetic field X: " << mag_x << endl;
        cout << "Magnetic field X: " << mag_x << endl << endl;
        cout << "ZMP_Loli = (" << X << ", " << _yzmp << ") [mm]" << endl;
        cout << "ZMP_Juanlo: (" << Xzmp << ", " << Yzmp << ") [mm]" << endl << endl;
        cout << "Iteration time: " << act_loop*1000 << " ms" << endl;
        cout << "Time between iterations: " << it_time*1000 << " ms" << endl;
        cout << "Absolute time: " << act_time << " s" << endl;

    }

    void saveInFileTxt()
    {
        ofstream out;

        out.open("data_ZMP.txt",ios::app);
        out << "Time: " << act_time << " ";
        out << "Xcom_Juanlo: " << Xcom << " ";
        out << "Ycom_Juanlo: " << Ycom << " ";
        out << "Zcom_Juanlo: " << Zcom << " ";
        out << "x_robot_Juanlo: " << x_robot << " ";
        out << "y_robot_Juanlo: " << y_robot << " ";
        out << "z_robot_Juanlo: " << z_robot << " ";
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
            fprintf(fp,"Tiempo,Xcom,Ycom,Zcom,x_robot,y_robot,z_robot,angulo_x,angulo_y,angulo_z,aceleracion_x,aceleracion_y,aceleracion_z");
            fprintf(fp,",velocidad_angular_x,velocidad_angular_y,velocidad_angular_z,campo_magnetico_x,campo_magnetico_y,campo_magnetico_z");
            fprintf(fp,",Xzmp_Juanlo,Yzmp_Juanlo,fx0,fy0,fz0,mx0,my0,mz0,Xzmp0,Yzmp0,fx1,fy1,fz1,mx1,my1,mz1,Xzmp1,Yzmp1,Xzmp_Loli,Yzmp_Loli");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%i", Xcom);
        fprintf(fp,",%i", Ycom);
        fprintf(fp,",%.2f", Zcom);
        fprintf(fp,",%.2f", x_robot);
        fprintf(fp,",%.2f", y_robot);
        fprintf(fp,",%.2f", z_robot);
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
        fprintf(fp,",%.2f", Xzmp);
        fprintf(fp,",%.2f", Yzmp);
        fprintf(fp,",%.2f", _fx0);
        fprintf(fp,",%.2f", _fy0);
        fprintf(fp,",%.2f", _fz0);
        fprintf(fp,",%.2f", _mx0);
        fprintf(fp,",%.2f", _my0);
        fprintf(fp,",%.2f", _mz0);
        fprintf(fp,",%.2f", _xzmp0);
        fprintf(fp,",%.2f", _yzmp0);
        fprintf(fp,",%.2f", _fx1);
        fprintf(fp,",%.2f", _fy1);
        fprintf(fp,",%.2f", _fz1);
        fprintf(fp,",%.2f", _mx1);
        fprintf(fp,",%.2f", _my1);
        fprintf(fp,",%.2f", _mz1);
        fprintf(fp,",%.2f", _xzmp1);
        fprintf(fp,",%.2f", _yzmp1);
        fprintf(fp,",%.2f", X);
        fprintf(fp,",%.2f", _yzmp);
    }



private:
    int n;
    double acc_x, acc_y, acc_z, x, y, z, x_robot, y_robot, z_robot, Xzmp, Yzmp; // x, y, z en [cm]
    double ang_x, ang_y, ang_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z;
    double init_time, act_time, init_loop, act_loop, it_time, it_prev;
    deque<double> x_sensor, y_sensor, z_sensor;

    float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
    float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]

    float e; // distance [m] between ground and sensor center
    float offs_x_j, offs_x_l; // zmp offset in initial time.
    float offs_y;
    float sum_j, sum_l;
    float _xzmp0, _yzmp0; // ZMP sensor 0
    float _xzmp1, _yzmp1; // ZMP sensor 1
    float _xzmp; // Global x_ZMP
    float _yzmp; // Global y_ZMP
    float X;

};

#endif
