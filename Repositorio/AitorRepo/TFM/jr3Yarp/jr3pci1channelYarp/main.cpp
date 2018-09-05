#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <yarp/os/all.h>

#include "jr3pci-ioctl.h"

int main(void) {

    yarp::os::Network yarp;
    yarp::os::Port port0;
    port0.open("/jr3ch0:o");

    six_axis_array fm;
    force_array fs;
    int ret, fd;
    int f[3], m[3]; // F-T from the sensor in cN (100*N) and cN*m (100*N*m) 
                    // Jr3 provides Forces in Newton and Torques in dNewton*meter. Scaled to get accuracy.
    float fx, fy, fz, mx, my, mz; // F-T in N and N*m
    if ((fd=open("/dev/jr3",O_RDWR)) < 0) {
        perror("Can't open device. No way to read force!");
    }

    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs);
    printf("Full scales are %d %d %d %d %d %d\n",fs.f[0],fs.f[1],fs.f[2],fs.m[0],fs.m[1],fs.m[2]);
    ret=ioctl(fd,IOCTL0_JR3_ZEROOFFS);

    while (1) {
        ret=ioctl(fd,IOCTL0_JR3_FILTER0,&fm);

        if (ret!=-1) {
            printf("Reading device ...");
            yarp::os::Bottle b;

            f[0] = 100*fm.f[0]*fs.f[0]/16384;
            f[1] = 100*fm.f[1]*fs.f[1]/16384;
            f[2] = 100*fm.f[2]*fs.f[2]/16384;

            m[0] = 10*fm.m[0]*fs.m[0]/16384;
            m[1] = 10*fm.m[1]*fs.m[1]/16384;
            m[2] = 10*fm.m[2]*fs.m[2]/16384;

            printf("Sensor data :\n");
            printf("[%d,%d,%d,%d,%d,%d]\n",f0,f1,f2,m0,m1,m2);
            // Other sensor data:
            //printf("%d\n",(short)fs.v[1]/16384);
            //printf("%d\n",(short)fs.v[2]/16384);

            fx = (float) f[0]/100;
            fy = (float) f[1]/100;
            fz = (float) f[2]/100;
            mx = (float) m[0]/100;
            my = (float) m[1]/100;
            mz = (float) m[2]/100;

            printf("F = [%f, %f, %f] N\n", fx,fy,fz);
            printf("M = [%f, %f, %f] N·m\n", mx,my,mz);

            /** SEND DATA **/
            b.addDouble(fx);
            b.addDouble(fy);
            b.addDouble(fz);
            b.addDouble(mx);
            b.addDouble(my);
            b.addDouble(mz);
            port0.write(b);

        } else perror("");

        //Sample time = 1ms
        usleep(100000); // delay in microseconds
    }
    close(fd);
}

