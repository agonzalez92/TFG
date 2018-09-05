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
#define pi 3.14159265358979323846
#define g 9.81  // Gravity in m/s²
#define TS 0.03
#define L   892.7 // Pendulum Longitude
#define M   62.589 // Robot mass

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
//#define Zcom 103.6602 //Distance to COM in Z axis [cm]
#define Zcom 89.27 //Distance to COM in Z axis [cm]
//Low-pass Filter
#define samples 300       //Number of samples for computing average
