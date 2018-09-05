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
#define L   0.8927 // Pendulum Longitude
#define M   62.589 // Robot mass

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 103.6602 //Distance to COM in Z axis [cm]

//Low-pass Filter
#define samples 20       //Number of samples for computing average

#include "ratethread.h"

int main(int argc, char *argv[])
{
    float ang=0;
    fp = fopen("../data_zmp.csv","w+");

    //INITIALISE YARP
    Network yarp;
    if ( ! yarp.checkNetwork() ){
        cerr << "[error] YARP network not found." << endl;
        return -1;
    } else cout << "[success] YARP network found." << endl;

    /** Opening YARP ports**/
    portImu.open("/inertial:i");
    port0.open("/jr3ch0:i");
    port1.open("/jr3ch1:i");
    Time::delay(0.5);

    //CONNECT TO FT-SENSORS
    yarp.connect("/jr3/ch0:o","/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    Time::delay(0.5);
    yarp.connect("/jr3/ch1:o","/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    Time::delay(0.5);
    yarp.connect("/inertial", "/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    Time::delay(0.5);


    /** LOOP THREAD **/
    MyRateThread myRateThread;
    myRateThread.start();

    //WAIT FOR ENTER AND EXIT LOOP
    char c;
    do {
        c=getchar();
    } while(c != '\n');
    myRateThread.stop();
    Time::delay(0.5);

    //CLOSE PORTS AND DEVICES
    port0.close();
    port1.close();
    portImu.close();

    return 0;
}
