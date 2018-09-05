//
//  main.cpp
//  
//
//  Created by Loli Pinel del Valle on 3/5/16.
//
//

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

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace std;

yarp::os::Port port0;
yarp::os::Port port1;
float fx0, fy0, fz0, mx0, my0, mz0;
float fx1, fy1, fz1, mx1, my1, mz1;
float e = 0.194;  // distance [m] between ground and sensor center
float xzmp0, yzmp0; // Right foot ZMP
float xzmp1, yzmp1; // Left foot ZMP
float xzmp, yzmp; // Global ZMP

int main(void) {
    yarp::os::Network yarp;
    
    /** Check yarp network**/
    printf("Checking network...\n");
    if (!yarp.checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }
    printf("Network ok\n");
    
    /** Opening YARP ports**/
    port0.open("/jr3ch0:i");
    port1.open("/jr3ch1:i");
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch0:o","/jr3ch0:i");
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch1:o","/jr3ch1:i");
    
    while (1) {
        yarp::os::Bottle b0;
        yarp::os::Bottle b1;
        
        port0.read(b0);
        port1.read(b1);
        
        fx0 = b0.get(0).asDouble();
        fy0 = b0.get(1).asDouble();
        fz0 = b0.get(2).asDouble();
        mx0 = b0.get(3).asDouble();
        my0 = b0.get(4).asDouble();
        mz0 = b0.get(5).asDouble();
        
        fx1 = b1.get(0).asDouble();
        fy1 = b1.get(1).asDouble();
        fz1 = b1.get(2).asDouble();
        mx1 = b1.get(3).asDouble();
        my1 = b1.get(4).asDouble();
        mz1 = b1.get(5).asDouble();
        
        /** ZMP Equations : Double Support **/
        xzmp0 = -(my0 + e*fx0) / fz0;
        yzmp0 = (mx0 + e*fy0) / fz0;
        
        xzmp1 = -(my1 + e*fx1) / fz1;
        yzmp1 =( mx1 + e*fy1) / fz1;
        
        xzmp = (xzmp0 * fz0 + xzmp1 * fz1) / (fz0 + fz1);
        yzmp = (yzmp0 * fz0 + yzmp1 * fz1) / (fz0 + fz1);
        
        xzmp = xzmp * 1000; //in mm
        yzmp = yzmp * 1000;
        
        if ((xzmp != xzmp) || (yzmp != yzmp)){
            printf ("Warning: No zmp data\n");
        } else {
        cout << "ZMP = [" << xzmp << ", " << yzmp << "]" << endl;
        }
    }

}
