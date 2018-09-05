//#include "global.h"
#include "ratethread.h"

int main(int argc, char *argv[])
{

    fp = fopen("../data_zmp.csv","w+");

    //-- INITIALISE YARP
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() ){
        cerr << "[error] YARP network not found." << endl;
        return -1;
    } else cout << "[success] YARP network found." << endl;

    //-- OPEN YARP PORTS
    portImu.open("/inertial:i");
    port0.open("/jr3ch0:i");
    port1.open("/jr3ch1:i");
    yarp::os::Time::delay(0.5);

    //-- CONNECT TO FT-SENSORS
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

    std::string bodyExecutionStr("/bodyExecution");

    // ------ LEFT LEG CONF -------
    yarp::os::Property leftLegOptions;
    leftLegOptions.put("device","remote_controlboard");
    leftLegOptions.put("remote","/teo/leftLeg");
    leftLegOptions.put("local",bodyExecutionStr+"/teo/leftLeg");
    leftLegDevice.open(leftLegOptions);
    if(!leftLegDevice.isValid()) {
      printf("robot leftLeg device not available.\n");
      leftLegDevice.close();
      yarp::os::Network::fini();
      return false;
    }
    if (!leftLegDevice.view(leftLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftLegIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIControlMode2 interface\n");
    if (!leftLegDevice.view(leftLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIPositionControl2 interface\n");

    // ------ RIGHT LEG CONF -------
    yarp::os::Property rightLegOptions;
    rightLegOptions.put("device","remote_controlboard");
    rightLegOptions.put("remote","/teo/rightLeg");
    rightLegOptions.put("local",bodyExecutionStr+"/teo/rightLeg");
    rightLegDevice.open(rightLegOptions);
    if(!rightLegDevice.isValid()) {
      printf("robot rightLeg device not available.\n");
      rightLegDevice.close();
      yarp::os::Network::fini();
      return false;
    }
    if (!rightLegDevice.view(rightLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring rightLegIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIControlMode2 interface\n");
    if (!rightLegDevice.view(rightLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIPositionControl2 interface\n");

    // ------ TRUNK CONF -------
    yarp::os::Property trunkOptions;
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("remote","/teo/trunk");
    trunkOptions.put("local",bodyExecutionStr+"/teo/trunk");
    trunkDevice.open(trunkOptions);
    if(!trunkDevice.isValid()) {
      printf("robot trunk device not available.\n");
      trunkDevice.close();
      yarp::os::Network::fini();
      return false;
    }
    if (!trunkDevice.view(trunkIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring trunkIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired trunkIControlMode2 interface\n");
    if (!trunkDevice.view(trunkIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring trunkIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired trunkIPositionControl2 interface\n");


    //-- SET CONTROL MODE
    int leftLegAxes;
    leftLegIPositionControl2->getAxes(&leftLegAxes);
    std::vector<int> leftLegControlModes(leftLegAxes,VOCAB_CM_POSITION);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-Leg\n");
        return false;
    }

    int rightLegAxes;
    rightLegIPositionControl2->getAxes(&rightLegAxes);
    std::vector<int> rightLegControlModes(rightLegAxes,VOCAB_CM_POSITION);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-Leg\n");
        return false;
    }

    int trunkAxes;
    trunkIPositionControl2->getAxes(&trunkAxes);
    std::vector<int> trunkControlModes(trunkAxes,VOCAB_CM_POSITION);
    if(! trunkIControlMode2->setControlModes(trunkControlModes.data())){
        printf("[warning] Problems setting position control mode of: trunk\n");
        return false;
    }

    yarp::os::Time::delay(1);

    //-- LOOP THREAD
    MyRateThread myRateThread;
    myRateThread.start();

    //-- WAIT FOR ENTER AND EXIT LOOP
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
