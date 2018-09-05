#include <iostream>
#include <fstream>
#include <math.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <stdio.h>

#define PI 3.1416;

using namespace yarp::os;

using namespace std;

int main(){

    double x;
    double y;

    Network yarp;
    Bottle bot;
    Port output;
    output.open("/write");
    while(1) {
        //printf("Got message: %s\n", bot.toString().c_str());
        cout << "Introduce la coordenada x del punto" << endl;
        cin >> x;
        cout << "Introduce la coordenada y del punto" << endl;
        cin >> y;
        double suma = pow(x,2) + pow(y,2);
        double division = x/y;

        double r = sqrt(suma);
        double ang = atan(1/division) * 180 / PI;

        bot.clear();
        bot.addDouble(x);
        bot.addDouble(y);
        cout << r << " " << ang << endl;
        output.write(bot);
        // Now exit the loop if first element (this is, 0), treated as a string, equals "quit":
        if(bot.get(0).asString() == "quit") break;
    }

    output.close();
    return 0;
}
