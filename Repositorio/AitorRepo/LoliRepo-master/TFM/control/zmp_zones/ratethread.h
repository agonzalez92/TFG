#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"

static FILE *fp;

yarp::os::Port port0;
yarp::os::Port port1;
yarp::dev::IPositionControl *posRightLeg;
yarp::dev::IPositionControl *posLeftLeg;


class MyRateThread : public yarp::os::RateThread
{
public:
    MyRateThread() : yarp::os::RateThread(TS*1000.0){
        n = 1;
	sum = 0.0;
	e = 0.194;

    }
    void run(){
        printf("----------\n Running\n");
        _dt = n*TS;
    /**
	// RAMP REFERENCES NO BACK TO ZERO. FORWARDS
        if (n <=  300){ref = 0.0;}
        else if (n >= 300 && n <= 330){ref = (0.02/30)*n-0.2;}
        else if (n >= 600 && n <= 630){ref = (0.02/30)*n-0.38;}
        else if (n >= 900 && n <= 930){ref = (0.02/30)*n-0.56;}
        else if (n >= 1200 && n <= 1230){ref = (0.02/30)*n-0.74;}
        else if (n >= 1500 && n <= 1530){ref = (0.02/30)*n-0.92;}
        else if (n >= 1800 && n <= 1830){ref = (0.02/30)*n-1.1;}
        else if (n >= 2100 && n <= 2130){ref = (0.02/30)*n-1.28;}
        else if (n >= 2400 && n <= 2430){ref = (0.02/30)*n-1.46;}
        else if (n >= 2700 && n <= 2730){ref = (0.02/30)*n-1.64;}
        else{ref = ref;} **/

   /**
	// RAMP REFERENCES BACK TO ZERO. SAME REACH TIME DIFFERENT SLOPE. FORWARDS
        if (n <= 300){ref = 0.0;}
        else if (n >= 300 && n <= 330){ref = (0.01/30)*n - 0.1;}
        else if (n >= 600 && n <= 630){ref = -(0.01/30)*n + 0.21;}
        else if (n >= 900 && n <= 930){ref = (0.02/30)*n - 0.6;}
        else if (n >= 1200 && n <= 1230){ref = -(0.02/30)*n + 0.82;}
        else if (n >= 1500 && n <= 1530){ref = (0.03/30)*n - 1.5;}
        else if (n >= 1800 && n <= 1830){ref = -(0.03/30)*n + 1.83;}
        else if (n >= 2100 && n <= 2130){ref = (0.04/30)*n - 2.8;}
        else if (n >= 2400 && n <= 2430){ref = -(0.04/30)*n + 3.24;}
        else if (n >= 2700 && n <= 2730){ref = (0.05/30)*n - 4.5;}
        else if (n >= 3000 && n <= 3030){ref = -(0.05/30)*n + 5.05;}
        else if (n >= 3300 && n <= 3330){ref = (0.06/30)*n - 6.6;}
        else if (n >= 3600 && n <= 3630){ref = -(0.06/30)*n + 7.26;}
        else {ref = ref;} **/

	/**
	// RAMP REF BACK TO ZERO. SAME SLOPE. FORWARDS
        if (n <= 300){ref = 0.0;}
        else if (n >= 300 && n <= 330){ref = (0.01/30)*n - 0.1;}
        else if (n >= 600 && n <= 630){ref = -(0.01/30)*n + 0.21;}
        else if (n >= 900 && n <= 960){ref = (0.02/60)*n - 0.3;}
        else if (n >= 1200 && n <= 1260){ref = -(0.02/60)*n + 0.42;}
        else if (n >= 1500 && n <= 1590){ref = (0.03/90)*n - 0.5;}
        else if (n >= 1800 && n <= 1890){ref = -(0.03/90)*n + 0.63;}
        else if (n >= 2100 && n <= 2220){ref = (0.04/120)*n - 0.7;}
        else if (n >= 2400 && n <= 2520){ref = -(0.04/120)*n + 0.84;}
        else {ref = ref;} **/

/**	// RAMPO REF BACK TO ZERO. LOW INCREMENT. FORWARDS
        if (n <= 300){ref = 0.0;}
        else if (n >= 300 && n <= 330){ref = (0.005/30)*n - 0.05;}
        else if (n >= 600 && n <= 630){ref = -(0.005/30)*n + 0.105;}
        else if (n >= 900 && n <= 930){ref = (0.01/30)*n - 0.3;}
        else if (n >= 1200 && n <= 1230){ref = -(0.01/30)*n + 0.41;}
        else if (n >= 1500 && n <= 1530){ref = (0.015/30)*n - 0.75;}
        else if (n >= 1800 && n <= 1830){ref = -(0.015/30)*n + 0.915;}
        else if (n >= 2100 && n <= 2130){ref = (0.02/30)*n - 1.4;}
        else if (n >= 2400 && n <= 2430){ref = -(0.02/30)*n + 1.62;}
        else if (n >= 2700 && n <= 2730){ref = (0.025/30)*n - 2.25;}
        else if (n >= 3000 && n <= 3030){ref = -(0.025/30)*n + 2.525;}
        else if (n >= 3300 && n <= 3330){ref = (0.03/30)*n - 3.3;}
        else if (n >= 3600 && n <= 3630){ref = -(0.03/30)*n + 3.63;}
        else {ref = ref;}**/

	// RAMP REFERENCES. BACKWARDS
	if (n <= 300){ref = 0.0;}
        else if (n >= 300 && n <= 330){ref = -(0.005/30)*n + 0.05;}
        else if (n >= 600 && n <= 630){ref = (0.005/30)*n - 0.105;}
        else if (n >= 900 && n <= 930){ref = -(0.01/30)*n + 0.3;}
        else if (n >= 1200 && n <= 1230){ref = (0.01/30)*n - 0.41;}
        else if (n >= 1500 && n <= 1530){ref = -(0.015/30)*n + 0.75;}
        else if (n >= 1800 && n <= 1830){ref = (0.015/30)*n - 0.915;}
        else if (n >= 2100 && n <= 2130){ref = -(0.02/30)*n + 1.4;}
        else if (n >= 2400 && n <= 2430){ref = (0.02/30)*n - 1.62;}
        else if (n >= 2700 && n <= 2730){ref = -(0.025/30)*n + 2.25;}
        else if (n >= 3000 && n <= 3030){ref = (0.025/30)*n - 2.525;}
        else if (n >= 3300 && n <= 3330){ref = -(0.03/30)*n + 3.3;}
        else if (n >= 3600 && n <= 3630){ref = (0.03/30)*n - 3.63;}

	else {ref = ref;}

        getInitialTime();
        readFTSensor();
        zmpComp();
        evaluateModel();
        setJoints();
        printData();
        saveToFile();
        n++;
        getCurrentTime();
        double t = curr_time - init_loop;  
        printf("LoopTime = %f\n",t);
}
    void getInitialTime()
    {
        init_loop = yarp::os::Time::now();
    }
    void getCurrentTime(){
        curr_time = yarp::os::Time::now();
    }

    void readFTSensor(){
        yarp::os::Bottle b0;
        yarp::os::Bottle b1;
        port0.read(b0);
        port1.read(b1);

        _fx0 = b0.get(0).asDouble();
        _fy0 = b0.get(1).asDouble();
        _fz0 = b0.get(2).asDouble();
        _mx0 = b0.get(3).asDouble();
        _my0 = b0.get(4).asDouble();

        _fx1 = b1.get(0).asDouble();
        _fy1 = b1.get(1).asDouble();
        _fz1 = b1.get(2).asDouble();
        _mx1 = b1.get(3).asDouble();
        _my1 = b1.get(4).asDouble();
    }

    void zmpComp(){
        /** ZMP Equations : Double Support **/
        _xzmp0 = -(_my0 + e*_fx0) / _fz0;
        _yzmp0 = (_mx0 + e*_fy0) / _fz0;

        _xzmp1 = -(_my1 + e*_fx1) /_fz1;
        _yzmp1 = (_mx1 + e*_fy1) /_fz1;

        _xzmp = (_xzmp0 * _fz0 + _xzmp1 * _fz1) / (_fz0 + _fz1);
        _yzmp = (_yzmp0 * _fz0 + _yzmp1 * _fz1) / (_fz0 + _fz1);

        // OFFSET

	if (n >=0 && n < 10){
		sum = _xzmp + sum;
		offs_x = sum / n;
	}
        _xzmp = (_xzmp - offs_x);
        _yzmp = _yzmp - offs_y;

        if ((_xzmp != _xzmp) || (_yzmp != _yzmp)){
            printf ("Warning: No zmp data\n");
        }
    }

    void evaluateModel(){
        /** EVALUACION MODELO **/
        _eval_x.model(_xzmp, ref);
       // _eval_y.model(_yzmp);

        angle_x = -1.6*asin(_eval_x.y/1.03)*180/PI;
        // angle_y = 90-(acos(_eval_y.y/1.03)*180/PI);

    }
    void setJoints(){
        posRightLeg->positionMove(4, angle_x);
        posLeftLeg->positionMove(4, angle_x);
//        posRightLeg->positionMove(5, -angle_y); // axial ankle Right Leg
//        posRightLeg->positionMove(1, -angle_y); // axial hip Right Leg
//        posLeftLeg->positionMove(5, angle_y); // axial ankle Left Leg
//        posLeftLeg->positionMove(1, angle_y); // axial hip Left Leg
    }
    void printData(){
        cout << "t = " << _dt << endl;
/**        cout << "ref = " << _eval_x._zmp_ref << endl;
        cout << "ZMP = [" << _xzmp << ", " << _yzmp << "]" << endl;
        cout << "Azmp = " << _eval_x._zmp_error << endl;
        cout << "x_model = " << _eval_x.y << endl;
        cout << "x1[0] = " << _eval_x._x1[0] << endl;
        cout << "Ud = " << _eval_x._u_ref << endl;
        cout << "u = " << _eval_x._u << endl;
        cout << "angle_x = " << angle_x << endl;**/
    }
    void saveToFile()
    {
          fprintf(fp,"\n%d", n);
          fprintf(fp,",%.4f",_dt);
          fprintf(fp,",%.15f", _xzmp);
          fprintf(fp,",%.15f", _eval_x.y);
          fprintf(fp,",%.15f", _eval_x._x1[0]);
          fprintf(fp,",%.15f", _eval_x._zmp_ref);
          fprintf(fp,",%f", _eval_x._u);
          fprintf(fp,",%f", angle_x);

//        fprintf(fp,",%.15f", _eval_y._r);
//        fprintf(fp,",%.15f", _yzmp);
//        fprintf(fp,",%.15f", _eval_y.y);
//        fprintf(fp,",%f", angle_y);
    }

private:
    int n;
    LIPM2d _eval_x;
    //LIPM2d _eval_y;
    float _fx0, _fy0, _fz0, _mx0, _my0; // F-T from sensor 0
    float _fx1, _fy1, _fz1, _mx1, _my1; // F-T from sensor 1


    float e; // distance [m] between ground and sensor center
    float offs_x; // zmp offset in initial time.
    float offs_y;
    float sum;
    float _xzmp0, _yzmp0; // ZMP sensor 0
    float _xzmp1, _yzmp1; // ZMP sensor 1
    float _xzmp; // Global x_ZMP
    float _yzmp; // Global y_ZMP

    float angle_x;
    float angle_y;

    double init_time, init_loop, curr_time, _dt;

    float ref;
};

#endif //_ratethread_H_
