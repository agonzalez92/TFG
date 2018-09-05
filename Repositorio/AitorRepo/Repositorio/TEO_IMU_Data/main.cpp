// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*  Aitor González                                                             */
/*  TFG (UC3M)                                                   */

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdio.h>
#include <stdlib.h>
#include <deque>


using namespace std;
using namespace yarp::os;


int main()
{
    double ang_x, ang_y, ang_z, acc_x, acc_y, acc_z,
            spd_x, spd_y, spd_z, mag_x, mag_y, mag_z;
    deque<double> x_sensor(10), y_sensor(10), z_sensor(10);

    // Initialise YARP
    Network yarp;

    // Create a port
    Port readPort;

    // Open the port
    readPort.open("/usb:i");

    // Connect output and input ports
    Network::connect("/inertial", "/usb:i");
    printf("Connection stablished\n\n");

    Bottle bot;


    while (true){

        readPort.read(bot);

        // SEND DATA
        for (int i=0; i<bot.size(); i++)
        {
            // i = 0,1,2 --> euler angles (deg)
            // i = 3,4,5 --> linear acceleration (m/s²)
            // i = 6,7,8 --> angular speed (deg/s)
            // i = 9,10,11 --> magnetic field (arbitrary units)

            switch (i)
            {
            case 0:
                ang_x=bot.get(i).asDouble();
                cout << "Angle X: " << ang_x << " deg" << endl;
                break;
            case 1:
                ang_y=bot.get(i).asDouble();
                cout << "Angle Y: " << ang_y << " deg" << endl;
                break;
            case 2:
                ang_z=bot.get(i).asDouble();
                cout << "Angle Z: " << ang_z << " deg" << endl;
                break;
            case 3:
                acc_x=bot.get(i).asDouble();
                cout << "Acceleration X: " << acc_x << " m/s²" << endl;
                x_sensor.push_front(acc_x);
                x_sensor.pop_back();
                break;
            case 4:
                acc_y=bot.get(i).asDouble();
                cout << "Acceleration Y: " << acc_y << " m/s²" << endl;
                y_sensor.push_front(acc_y);
                y_sensor.pop_back();
                break;
            case 5:
                acc_z=bot.get(i).asDouble();
                cout << "Acceleration Z: " << acc_z << " m/s²" << endl;
                z_sensor.push_front(acc_z);
                z_sensor.pop_back();
                break;
            case 6:
                spd_x=bot.get(i).asDouble();
                cout << "Angular speed X: " << spd_x << " deg/s" << endl;
                break;
            case 7:
                spd_y=bot.get(i).asDouble();
                cout << "Angular speed Y: " << spd_y << " deg/s" << endl;
                break;
            case 8:
                spd_z=bot.get(i).asDouble();
                cout << "Angular speed Z: " << spd_z << " deg/s" << endl;
                break;
            case 9:
                mag_x=bot.get(i).asDouble();
                cout << "Magnetic field X: " << mag_x << endl;
                break;
            case 10:
                mag_y=bot.get(i).asDouble();
                cout << "Magnetic field Y: " << mag_y << endl;
                break;
            case 11:
                mag_z=bot.get(i).asDouble();
                cout << "Magnetic field Z: " << mag_z << endl;
                break;
            }
        }
        printf("\n");
    }

    //CLOSE PORTS AND DEVICES
    readPort.close();

    return 0;
}

