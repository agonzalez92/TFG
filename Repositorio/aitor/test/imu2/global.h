#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <cmath>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//General constants
#define PI  3.14159265358979323846
#define G   9.81  // Gravity in m/s²
#define TS  0.03
#define L   0.8927 // Pendulum Longitude [m]
#define M   62.589 // Robot mass [kg]

//Low-pass Filter
#define samples 50 //Number of samples for computing average

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 0.683 //Distance to COM in Z axis [m] - Zcom 103.6602cm JUANLO - Zcom 89.27cm LOLI - Zcom 0.683m ADJUST-JM

//PID parameters
#define dt 0.05 //Loop interval time [s]
#define max 10 //Maximum output value
#define min -10 //Minimum output value
//Ankle parameters
#define Kp_ankle 0.1 //Proportional gain
#define Kd_ankle 0.01 //Derivative gain
#define Ki_ankle 0.001 //Integral gain
//Hip parameters
#define Kp_hip 1 //Proportional gain
#define Kd_hip 0.01 //Derivative gain
#define Ki_hip 0.001 //Integral gain

