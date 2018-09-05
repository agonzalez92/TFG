/*******************************************************
 * main.cpp
 * @Brief: Program for reading forces and torques form two jr3 F/T sensors using jr3pcidriver.
 *      The information read from the sensors is written in a YARP port.
 * @Author: Loli Pinel
 * @Date: September 2015
 *******************************************************/


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>

#include <yarp/os/all.h>

#include "jr3pci-ioctl.h"

int main(void) {
 
    yarp::os::Network yarp;
    yarp::os::Port port0, port1, port2, port3;

    double init;
    double end;
    double t;

    six_axis_array fm0, fm1, fm2, fm3;
    force_array fs0, fs1, fs2, fs3;
    int ret, fd;
    int f0[3], m0[3]; // F-T from the sensor 0 in cN (100*N) and cN*m (100*N*m)
    int f1[3], m1[3]; // F-T from the sensor 1 in cN (100*N) and cN*m (100*N*m)
    int f2[3], m2[3]; // F-T from the sensor 2 in cN (100*N) and cN*m (100*N*m)
    int f3[3], m3[3]; // F-T from the sensor 3 in cN (100*N) and cN*m (100*N*m)
    // Jr3 provides Forces in Newton and Torques in dNewton*m. Scaled to get accuracy.
    float fx0, fy0, fz0, mx0, my0, mz0; // F-T from sensor 0 in N and N*m
    float fx1, fy1, fz1, mx1, my1, mz1; // F-T from sensor 1 in N and N*m
    float fx2, fy2, fz2, mx2, my2, mz2; // F-T from sensor 2 in N and N*m
    float fx3, fy3, fz3, mx3, my3, mz3; // F-T from sensor 3 in N and N*m

    initscr(); // Init ncurses mode
    curs_set(0); // Hide cursor

    /** Check YARP Network **/
    if (!yarp.checkNetwork()) {
        perror("Please start a yarp name server first\n");
       return(-1);
    }

    /** Opening YARP ports **/
    port0.open("/jr3ch0:o");
    port1.open("/jr3ch1:o");
    port2.open("/jr3ch2:o");
    port3.open("/jr3ch3:o");

    /** Open device **/
    if ((fd=open("/dev/jr3",O_RDWR)) < 0) {
        perror("Can't open device. No way to read force!");
	return(-1);
    }

    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs0);
    mvprintw(0,0,"Full scales of Sensor 0 are %d %d %d %d %d %d\n",fs0.f[0],fs0.f[1],fs0.f[2],fs0.m[0],fs0.m[1],fs0.m[2]);
    ret=ioctl(fd,IOCTL1_JR3_GET_FULL_SCALES,&fs1);
    mvprintw(1,0,"Full scales of Sensor 1 are: %d %d %d %d %d %d\n", fs1.f[0],fs1.f[1],fs1.f[2],fs1.m[0],fs1.m[1],fs1.m[2]);
    ret=ioctl(fd,IOCTL2_JR3_GET_FULL_SCALES,&fs2);
    mvprintw(2,0,"Full scales of Sensor 2 are: %d %d %d %d %d %d\n", fs2.f[0],fs2.f[1],fs2.f[2],fs2.m[0],fs2.m[1],fs2.m[2]);
    ret=ioctl(fd,IOCTL3_JR3_GET_FULL_SCALES,&fs3);
    mvprintw(3,0,"Full scales of Sensor 3 are: %d %d %d %d %d %d\n", fs3.f[0],fs3.f[1],fs3.f[2],fs3.m[0],fs3.m[1],fs3.m[2]);

    ret=ioctl(fd,IOCTL0_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL1_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL2_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL3_JR3_ZEROOFFS);
    

    while (1) {
        init = yarp::os::Time::now();
        ret=ioctl(fd,IOCTL0_JR3_FILTER0,&fm0);
        ret=ioctl(fd,IOCTL1_JR3_FILTER0,&fm1);
        ret=ioctl(fd,IOCTL2_JR3_FILTER0,&fm2);
        ret=ioctl(fd,IOCTL3_JR3_FILTER0,&fm3);

        if (ret!=-1) {
            yarp::os::Bottle b0, b1, b2, b3;

            //printf("Reading device ...\n");

            // -------- SENSOR 0 ------------ //

            f0[0] = 100*fm0.f[0]*fs0.f[0]/16384;
            f0[1]= 100*fm0.f[1]*fs0.f[1]/16384;
            f0[2] = 100*fm0.f[2]*fs0.f[2]/16384;
            m0[0] = 10*fm0.m[0]*fs0.m[0]/16384;
            m0[1] = 10*fm0.m[1]*fs0.m[1]/16384;
            m0[2] = 10*fm0.m[2]*fs0.m[2]/16384;

            mvprintw(5,0,"Sensor 0 data :"); // Print at row 0, col 0
            mvprintw(6,0,"[%d,%d,%d,%d,%d,%d]\n",f0[0],f0[1],f0[2],m0[0],m0[1],m0[2]);
            // Other sensor data
            //printf("%d\n",(short)fs0.v[1]/16384);
            //printf("%d\n",(short)fs0.v[2]/16384);

            fx0 = (float) f0[0]/100;
            fy0 = (float) f0[1]/100;
            fz0 = (float) f0[2]/100;
            mx0 = (float) m0[0]/100;
            my0 = (float) m0[1]/100;
            mz0 = (float) m0[2]/100;

            mvprintw(7,0,"F0 = [%+f, %+f, %+f] N\n", fx0,fy0,fz0);
            mvprintw(8,0,"M0 = [%+f, %+f, %+f] N·m\n", mx0,my0,mz0);

            // -------- SENSOR 1 ------------ //
            f1[0] = 100*fm1.f[0]*fs1.f[0]/16384;
            f1[1] = 100*fm1.f[1]*fs1.f[1]/16384;
            f1[2] = 100*fm1.f[2]*fs1.f[2]/16384;
            m1[0] = 10*fm1.m[0]*fs1.m[0]/16384;
            m1[1] = 10*fm1.m[1]*fs1.m[1]/16384;
            m1[2] = 10*fm1.m[2]*fs1.m[2]/16384;

            mvprintw(9,0,"Sensor 1 data :");
            mvprintw(10,0,"[%d,%d,%d,%d,%d,%d]\n",f1[0],f1[1],f1[2],m1[0],m1[1],m1[2]);
            // Other sensor data
            //printf("%d\n",(short)fs1.v[1]/16384);
            //printf("%d\n",(short)fs1.v[2]/16384);

            fx1 = (float) f1[0]/100;
            fy1 = (float) f1[1]/100;
            fz1 = (float) f1[2]/100;
            mx1 = (float) m1[0]/100;
            my1 = (float) m1[1]/100;
            mz1 = (float) m1[2]/100;

            mvprintw(11,0,"F1 = [%+f, %+f, %+f] N\n", fx1,fy1,fz1);
            mvprintw(12,0,"M1 = [%+f, %+f, %+f] N·m\n", mx1,my1,mz1);

            // -------- SENSOR 2 ------------ //
            f2[0] = 100*fm2.f[0]*fs2.f[0]/16384;
            f2[1] = 100*fm2.f[1]*fs2.f[1]/16384;
            f2[2] = 100*fm2.f[2]*fs2.f[2]/16384;
            m2[0] = 10*fm2.m[0]*fs2.m[0]/16384;
            m2[1] = 10*fm2.m[1]*fs2.m[1]/16384;
            m2[2] = 10*fm2.m[2]*fs2.m[2]/16384;

            mvprintw(13,0,"Sensor 2 data :");
            mvprintw(14,0,"[%d,%d,%d,%d,%d,%d]\n",f2[0],f2[1],f2[2],m2[0],m2[1],m2[2]);
            // Other sensor data
            //printf("%d\n",(short)fs2.v[1]/16384);
            //printf("%d\n",(short)fs2.v[2]/16384);

            fx2 = (float) f2[0]/100;
            fy2 = (float) f2[1]/100;
            fz2 = (float) f2[2]/100;
            mx2 = (float) m2[0]/100;
            my2 = (float) m2[1]/100;
            mz2 = (float) m2[2]/100;

            mvprintw(15,0,"F2 = [%+f, %+f, %+f] N\n", fx2,fy2,fz2);
            mvprintw(16,0,"M2 = [%+f, %+f, %+f] N·m\n", mx2,my2,mz2);

            // -------- SENSOR 3 ------------ //
            f3[0] = 100*fm3.f[0]*fs3.f[0]/16384;
            f3[1] = 100*fm3.f[1]*fs3.f[1]/16384;
            f3[2] = 100*fm3.f[2]*fs3.f[2]/16384;
            m3[0] = 10*fm3.m[0]*fs3.m[0]/16384;
            m3[1] = 10*fm3.m[1]*fs3.m[1]/16384;
            m3[2] = 10*fm3.m[2]*fs3.m[2]/16384;

            mvprintw(17,0,"Sensor 3 data :");
            mvprintw(18,0,"[%d,%d,%d,%d,%d,%d]\n",f3[0],f3[1],f3[2],m3[0],m3[1],m3[2]);
            // Other sensor data
            //printf("%d\n",(short)fs3.v[1]/16384);
            //printf("%d\n",(short)fs3.v[2]/16384);

            fx3 = (float) f3[0]/100;
            fy3 = (float) f3[1]/100;
            fz3 = (float) f3[2]/100;
            mx3 = (float) m3[0]/100;
            my3 = (float) m3[1]/100;
            mz3 = (float) m3[2]/100;

            mvprintw(19,0,"F3 = [%+f, %+f, %+f] N\n", fx3,fy3,fz3);
            mvprintw(20,0,"M3 = [%+f, %+f, %+f] N·m\n", mx3,my3,mz3);

            /** SEND DATA **/
            b0.addDouble(fx0);
            b0.addDouble(fy0);
            b0.addDouble(fz0);
            b0.addDouble(mx0);
            b0.addDouble(my0);
            b0.addDouble(mz0);

            b1.addDouble(fx1);
            b1.addDouble(fy1);
            b1.addDouble(fz1);
            b1.addDouble(mx1);
            b1.addDouble(my1);
            b1.addDouble(mz1);

            b2.addDouble(fx2);
            b2.addDouble(fy2);
            b2.addDouble(fz2);
            b2.addDouble(mx2);
            b2.addDouble(my2);
            b2.addDouble(mz2);

            b3.addDouble(fx3);
            b3.addDouble(fy3);
            b3.addDouble(fz3);
            b3.addDouble(mx3);
            b3.addDouble(my3);
            b3.addDouble(mz3);

            port0.write(b0);
            port1.write(b1);
            port2.write(b2);
            port3.write(b3);

            refresh();
        } else perror("Could not read device\n");

        end = yarp::os::Time::now();
        t = end - init;
        mvprintw(21,0,"t=%f\n",t);
    }
    close(fd);
    endwin();
    return 0;
}

