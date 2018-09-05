/**
 * @Brief: Program for reading forces and torques form TWO jr3 F/T sensors using jr3pcidriver and computing ZMP.
 * @Author: Loli Pinel
 * @Date: February 2016
 **/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "jr3pci-ioctl.h"

int main(void) {

    six_axis_array fm0, fm1;
    force_array fs0, fs1;
    int ret, fd;
    int f00,f01,f02,m00,m01,m02; // F-T from the sensor 0 in Newton
    int f10,f11,f12,m10,m11,m12; // F-T from the sensor 1 in Newton
    float fx0, fy0, fz0, mx0, my0, mz0; // Scaled F-T from sensor 0 in dN*m (0.1N*m)
    float fx1, fy1, fz1, mx1, my1, mz1; // Scaled F-T from sensor 1 in dN*m (0.1N*m)
    float xzmp0, yzmp0; // ZMP sensor 0
    float xzmp1, yzmp1; // ZMP sensor 1
    float xzmp, yzmp; // Global ZMP

    if ((fd=open("/dev/jr3",O_RDWR)) < 0) {
        perror("Can't open device. No way to read force!");
    }

    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs0);
    printf("Full scales of Sensor 0 are %d %d %d %d %d %d\n",fs0.f[0],fs0.f[1],fs0.f[2],fs0.m[0],fs0.m[1],fs0.m[2]);
    ret=ioctl(fd,IOCTL1_JR3_GET_FULL_SCALES,&fs1);
    printf("Full scales of Sensor 1 are: %d %d %d %d %d %d\n", fs1.f[0],fs1.f[1],fs1.f[2],fs1.m[0],fs1.m[1],fs1.m[2]);
    ret=ioctl(fd,IOCTL0_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL1_JR3_ZEROOFFS);


    while (1) {
        ret=ioctl(fd,IOCTL0_JR3_FILTER0,&fm0);
        ret=ioctl(fd,IOCTL1_JR3_FILTER0,&fm1);

        if (ret!=-1) {
            printf("Reading device ...\n");

            // -------- SENSOR 0 ------------ //
            printf("Sensor 0 : [ ");

            f00 = 100*fm0.f[0]*fs0.f[0]/16384;
            f01 = 100*fm0.f[1]*fs0.f[1]/16384;
            f02 = 100*fm0.f[2]*fs0.f[2]/16384;
            m00 = 10*fm0.m[0]*fs0.m[0]/16384;
            m01 = 10*fm0.m[1]*fs0.m[1]/16384;
            m02 = 10*fm0.m[2]*fs0.m[2]/16384;

            printf("Sensor 0 data :");
            printf("[%d,%d,%d,%d,%d,%d]\n",f00,f01,f02,m00,m01,m02);
            // Other sensor data
            //printf("%d\n",(short)fs0.v[1]/16384);
            //printf("%d\n",(short)fs0.v[2]/16384);

            fx0 = (float) f00/100;
            fy0 = (float) f01/100;
            fz0 = (float) f02/100;
            mx0 = (float) m00/100;
            my0 = (float) m01/100;
            mz0 = (float) m02/100;

            printf("F0 = [%f, %f, %f] N\n", fx0,fy0,fz0);
            printf("M0 = [%f, %f, %f] N·m\n", mx0,my0,mz0);

            // -------- SENSOR 1 ------------ //
            f10 = 100*fm1.f[0]*fs1.f[0]/16384;
            f11 = 100*fm1.f[1]*fs1.f[1]/16384;
            f12 = 100*fm1.f[2]*fs1.f[2]/16384;
            m10 = 10*fm1.m[0]*fs1.m[0]/16384;
            m11 = 10*fm1.m[1]*fs1.m[1]/16384;
            m12 = 10*fm1.m[2]*fs1.m[2]/16384;

            printf("Sensor 1 data :");
            printf("[%d,%d,%d,%d,%d,%d]\n",f10,f11,f12,m10,m11,m12);
            // Other sensor data
            //printf("%d\n",(short)fs1.v[1]/16384);
            //printf("%d\n",(short)fs1.v[2]/16384);

            fx1 = (float) f10/100;
            fy1 = (float) f11/100;
            fz1 = (float) f12/100;
            mx1 = (float) m10/100;
            my1 = (float) m11/100;
            mz1 = (float) m12/100;

            printf("F1 = [%f, %f, %f] N\n", fx1,fy1,fz1);
            printf("M1 = [%f, %f, %f] N·m\n", mx1,my1,mz1);

            /** ZMP Equations : Double Support **/
            xzmp0 = -my0 / fz0;
            yzmp0 = mx0 / fz0;

            xzmp1 = -my1 /fz1;
            yzmp1 = mx1 /fz1;

            xzmp = (xzmp0 * fz0 + xzmp1 * fz1) / (fz0 + fz1);
            yzmp = (yzmp0 * fz0 + yzmp1 * fz1) / (fz0 + fz1);

            printf("ZMP = [ %f ,%f]\n", xzmp, yzmp);

        } else perror("");

        //Sample time = 1ms
        usleep(100000); // delay in microseconds
    }
    close(fd);
}

