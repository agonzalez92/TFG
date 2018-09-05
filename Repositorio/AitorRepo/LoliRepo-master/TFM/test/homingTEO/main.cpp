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

int main(void) {
    yarp::os::Network yarp;

    /** Check yarp network**/
    printf("Checking network...\n");
    if (!yarp.checkNetwork()) {
        printf("Please start a yarp name server first\n");
       return(-1);
    }
    printf("Network ok\n");

    /** SET CONFIG **/
    yarp::os::Property optionsLeftLeg;
    optionsLeftLeg.put("device","remote_controlboard");
    optionsLeftLeg.put("remote","/teo/leftLeg");
    optionsLeftLeg.put("local","/loli/leftLeg");
    yarp::dev::PolyDriver devLeftLeg(optionsLeftLeg);
    if(!devLeftLeg.isValid()) {
        printf("TEO device not available.\n");
        devLeftLeg.close();
        yarp::os::Network::fini();
        return 1;
    }

    yarp::dev::IPositionControl *posLeftLeg;
    bool okLL = devLeftLeg.view(posLeftLeg);
    if (!okLL) {
        printf("[warning] Problems acquiring robot IPositionControl LeftLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl LeftLeg interface\n");

    yarp::os::Property optionsRightLeg;
    optionsRightLeg.put("device","remote_controlboard");
    optionsRightLeg.put("remote","/teo/rightLeg");
    optionsRightLeg.put("local","/loli/rightLeg");

    yarp::dev::PolyDriver devRightLeg(optionsRightLeg);
    if(!devRightLeg.isValid()) {
        printf("TEO device not available.\n");
        devRightLeg.close();
        yarp::os::Network::fini();
        return 1;
    }
    yarp::dev::IPositionControl *posRightLeg;
    bool okRL = devRightLeg.view(posRightLeg);
    if (!okRL) {
        printf("[warning] Problems acquiring robot IPositionControl RightLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl RightLeg inteface\n");


    /** SET MODE **/
    printf("Set position mode Left Leg\n");
    posLeftLeg->setPositionMode();
    printf("Set position mode Right Leg\n");
    posRightLeg->setPositionMode();

    /** SET LEFT LEG TO 0 **/
    posLeftLeg->positionMove(0,0);
    posLeftLeg->positionMove(1,0);
    posLeftLeg->positionMove(2,0);
    posLeftLeg->positionMove(3,0);
    posLeftLeg->positionMove(4,0);
    posLeftLeg->positionMove(5,0);
    printf("Left Leg :(0 0 0 0 0 0)\n");
    /** SET RIGHT LEG TO 0 **/
    posRightLeg->positionMove(0,0);
    posRightLeg->positionMove(1,0);
    posRightLeg->positionMove(2,0);
    posRightLeg->positionMove(3,0);
    posRightLeg->positionMove(4,0);
    posRightLeg->positionMove(5,0);
    printf("Right Leg :(0 0 0 0 0 0)\n");

}
