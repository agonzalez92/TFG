#include "global.h"
#include "ratethread.h"



int main(int argc, char *argv[])
{

    fp = fopen("../data_zmp.csv","w+");

    //INITIALISE YARP
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() ){
        cerr << "[error] YARP network not found." << endl;
        return -1;
    } else cout << "[success] YARP network found." << endl;

    /** Opening YARP ports**/
    portImu.open("/inertial:i");
    port0.open("/jr3ch0:i");
    port1.open("/jr3ch1:i");
    yarp::os::Time::delay(0.5);

    //CONNECT TO FT-SENSORS
    yarp.connect("/jr3/ch0:o","/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3/ch1:o","/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    yarp::os::Time::delay(0.5);
    yarp.connect("/inertial", "/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);

    /** SET CONFIG LEFT LEG **/
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
    // Position control
    if (! devLeftLeg.view(posLeftLeg)) {
        printf("[warning] Problems acquiring robot IPositionControl leftLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl leftLeg interface\n");

    /** SET CONFIG RIGHT LEG **/
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
    // Position control
    if (! devRightLeg.view(posRightLeg)) {
        printf("[warning] Problems acquiring robot IPositionControl rightLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl rightLeg inteface\n");


    /** SET MODE **/
    /** Position Mode **/
    printf("Set position mode Left Leg\n");
    posLeftLeg->setPositionMode();
    printf("Set position mode Right Leg\n");
    posRightLeg->setPositionMode();

    yarp::os::Time::delay(1);

    /** LOOP THREAD **/
    MyRateThread myRateThread;
    myRateThread.start();

    //WAIT FOR ENTER AND EXIT LOOP
    char c;
    do {
        c=getchar();
    } while(c != '\n');
    myRateThread.stop();
    yarp::os::Time::delay(0.5);

    //CLOSE PORTS AND DEVICES
    port0.close();
    port1.close();
    portImu.close();

    return 0;
}
