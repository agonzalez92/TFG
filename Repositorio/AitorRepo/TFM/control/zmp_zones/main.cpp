#include "global.h"
#include "ratethread.h"

//yarp::os::Port port0;
//yarp::os::Port port1;

void controlC (void){
    fclose(fp);
    port0.close();
    port1.close();
    exit(-1);
}

int main(void) {
    yarp::os::Network yarp;

    fp = fopen("../data.csv","w+");

 //   signal (SIGINT, (__sighandler_t) controlC );

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

    bool okLL = devLeftLeg.view(posLeftLeg);
    if (!okLL) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testTEO acquired robot LeftArm interface\n");

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

    bool okRL = devRightLeg.view(posRightLeg);
    if (!okRL) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testTEO acquired robot RightArm inteface\n");


    /** SET MODE **/
    printf("Set position mode Left Leg\n");
    posLeftLeg->setPositionMode();
    printf("Set position mode Left Leg\n");
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

   
    yarp::os::Time::delay(10);
    /** LOOP THREAD**/
    MyRateThread myRateThread;
    myRateThread.start();

    char c;
    do {
        c=getchar();
    } while(c != '\n');
    myRateThread.stop();
    port0.close();
    port1.close();
    yarp::os::Time::delay(0.5); 
    

}
