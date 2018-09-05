#include <stdio.h>
#include <stdlib.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char *argv[]) {

    /** CONNECT YARP NETWORK **/
    Network yarp;
    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }

    /** SET CONFIG RIGHT ARM**/
    //	Property optionsRightArm;
    //	optionsRightArm.put("device","remote_controlboard");
    //	optionsRightArm.put("remote","/teo/rightArm");
    //	optionsRightArm.put("local","/loli/rightArm");
    //	PolyDriver rightArm(optionsRightArm);
    //	if(!rightArm.isValid()) {
    //		printf("TEO device not available.\n");
    //		rightArm.close();
    //		Network::fini();
    //		return 1;
    //	}

    /** SET CONFIG LEFT ARM **/
    //    Property optionsLeftArm;
    //    optionsLeftArm.put("device","remote_controlboard");
    //    optionsLeftArm.put("remote","/teo/leftArm");
    //    optionsLeftArm.put("local","/loli/leftArm");
    //    PolyDriver leftArm(optionsLeftArm);
    //    if(!leftArm.isValid()) {
    //        printf("TEO device not available.\n");
    //        leftArm.close();
    //        Network::fini();
    //        return 1;
    //    }

    /** SET CONFIG RIGHT LEG **/
    //    Property optionsRightLeg;
    //    optionsRightLeg.put("device","remote_controlboard");
    //    optionsRightLeg.put("remote","/teo/rightLeg");
    //    optionsRightLeg.put("local","/loli/rightLeg");
    //    PolyDriver rightLeg(optionsRightLeg);
    //    if(!rightLeg.isValid()) {
    //        printf("TEO device not available.\n");
    //        rightLeg.close();
    //        Network::fini();
    //        return 1;
    //    }

    /** SET CONFIG LEFT LEG **/
    Property optionsLeftLeg;
    optionsLeftLeg.put("device","remote_controlboard");
    optionsLeftLeg.put("remote","/teo/leftLeg");
    optionsLeftLeg.put("local","/loli/leftLeg");
    PolyDriver leftLeg(optionsLeftLeg);
    if(!leftLeg.isValid()) {
        printf("TEO device not available.\n");
        leftLeg.close();
        Network::fini();
        return 1;
    }

    /** SET POSITION MODE RIGHT ARM **/
    //	IPositionControl *posRightArm;
    //	bool okRA = rightArm.view(posRightArm);
    //	if (!okRA) {
    //		printf("[warning] Problems acquiring robot interface\n");
    //		return false;
    //	} else printf("[success] testTEO acquired robot RightArm inteface\n");
    //	posRightArm->setPositionMode();

    /** SET POSITION MODE LEFT ARM **/
    //	IPositionControl *posLeftArm;
    //	bool okLA = leftArm.view(posLeftArm);
    //	if (!okLA) {
    //		printf("[warning] Problems acquiring robot interface\n");
    //		return false;
    //	} else printf("[success] testTEO acquired robot LeftArm interface\n");
    //	posLeftArm->setPositionMode();

    /** SET POSITION MODE RIGHT LEG **/
    //    IPositionControl *posRightLeg;
    //    bool okRL = rightLeg.view(posRightLeg);
    //    if (!okRL) {
    //        printf("[warning] Problems acquiring robot interface\n");
    //        return false;
    //    } else printf("[success] testTEO acquired robot RightArm inteface\n");
    //    posRightLeg->setPositionMode();

    /** SET POSITION MODE LEFT LEG **/
    //    IPositionControl *posLeftLeg;
    //    bool okLL = leftLeg.view(posLeftLeg);
    //    if (!okLL) {
    //        printf("[warning] Problems acquiring robot interface\n");
    //        return false;
    //    } else printf("[success] testTEO acquired robot LeftArm interface\n");
    //    posLeftLeg->setPositionMode();

    /** SET VELOCITY MODE LEFT LEG **/
    IVelocityControl *velLeftLeg;
    bool okLL = leftLeg.view(velLeftLeg);
    if (!okLL) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testTEO acquired robot RightArm inteface\n");
    velLeftLeg->setVelocityMode();

    /** POSITION CONTROL. RIGHT AND LEFT ARMS**/
    //	printf("test Right Arm positionMove(0,10)\n");
    //	posRightArm->positionMove(0, 10);
    //	printf("test Left Arm positionMove(0,10)\n");
    //	posLeftArm->positionMove(0, 10);


    /** POSITION CONTROL. RIGHT AND LEFT LEGS**/
    //    printf("test Right Leg positionMove(2,-5)\n");
    //    posRightLeg->positionMove(2, -5);
    //    printf("test Left Leg positionMove(2,-10)\n");
    //    posLeftLeg->positionMove(2, -10);

    //    printf("Delaying 5 seconds ...\n");
    //    Time::delay(5);

    /** VELOCITY CONTROL. LEFT LEG **/
    printf("test Right Leg positionMove(2,-0.5)\n");
    velLeftLeg->velocityMove(2, -0.5);
    printf("Delaying 5 seconds ...\n");
    Time::delay(5);
    velLeftLeg->velocityMove(2, 0);




    //    IEncoders *encRightLeg;
    //    IEncoders *encLeftLeg;
    //    okRL = rightLeg.view(encRightLeg);
    //    okLL = leftLeg.view(encLeftLeg);
    //    IVelocityControl *vel;
    //    ok = dd.view(vel);
    //    vel->setVelocityMode();
    //    printf("test velocityMove(0,10)\n");
    //    vel->velocityMove(0,10);
    //
    //    printf("Delaying 5 seconds...\n");
    //    Time::delay(5);

    return 0;
}
