import yarp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PI = np.pi
d = 70+37.5
h = 0
e = 0.194  # distance [m] between ground and sensor center
def RightFoot():
    # Big semicircle
    ang = np.linspace(2*PI,PI,180)
    x0 = d + 70*np.cos(ang)
    y0 = h + 70*np.sin(ang)

    # Straightline
    x1 = np.ones(121) * (d-70)
    y1 = h + np.linspace(1,121,121) -1

    # Little circle
    ang2 = np.linspace(PI,14.25*PI/180,180)
    x2 = d - 15 + 55*np.cos(ang2)
    y2 = h + 120 + 55*np.sin(ang2)

    # Inclined line
    x3 = np.linspace(d - 15 + 55*np.cos(14.25*PI/180), d + 70*np.cos(2*PI), 100)
    y3 = np.linspace(h + 120 + 55*np.sin(14.25*PI/180), h + 70*np.sin(2*PI), 100)

    x_sole = np.concatenate([x0,x1,x2,x3])
    y_sole = np.concatenate([y0,y1,y2,y3])
    plt.plot(x_sole,y_sole,'k')

    return;

def LeftFoot():
    # Big semicircle
    ang = np.linspace(2*PI,PI,180)
    x0 = d + 70*np.cos(ang)
    y0 = h + 70*np.sin(ang)

    # Straightline
    x1 = np.ones(121) * (d-70)
    y1 = h + np.linspace(1,121,121) -1

    # Little circle
    ang2 = np.linspace(PI,14.25*PI/180,180)
    x2 = d - 15 + 55*np.cos(ang2)
    y2 = h + 120 + 55*np.sin(ang2)

    # Inclined line
    x3 = np.linspace(d - 15 + 55*np.cos(14.25*PI/180), d + 70*np.cos(2*PI), 100)
    y3 = np.linspace(h + 120 + 55*np.sin(14.25*PI/180), h + 70*np.sin(2*PI), 100)

    x_sole = np.concatenate([-x0,-x1,-x2,-x3])
    y_sole = np.concatenate([y0,y1,y2,y3])
    plt.plot(x_sole,y_sole,'k')

    return;

# Initialise YARP
yarp.Network.init()
# Create a port
p0 = yarp.Port()
p1 = yarp.Port()
# Open the port
p0.open("/gui0:i")
p1.open("/gui1:i")
# Connect output and input ports
yarp.Network.connect("/jr3ch0:o", "/gui0:i")
yarp.Network.connect("/jr3ch1:o", "/gui1:i")
# Read the data from de port
data0 = yarp.Bottle()
data1 = yarp.Bottle()

fig = plt.figure()
#fig.hold(True)

while 1:
    ax = fig.add_subplot(111)
    ax.grid()
    #ax.axis('equal')
    plt.xlim(300,-300)
    plt.ylim(-300,300)
    ax.set_title('ZMP REPRESENTATION IN DOUBLE SUPPORT', fontsize=12, fontweight='bold')
    ax.set_xlabel('y [mm]') # changed because of robot axes
    ax.set_ylabel('x [mm]') # changed because of robot axes

    # Plotting ZMP Areas
    RightFoot()
    LeftFoot()

    # Reading YARP port
    print "Reading..."
    p0.read(data0)
    p1.read(data1)

    fx0 = data0.get(0).asDouble()
    fy0 = data0.get(1).asDouble()
    fz0 = data0.get(2).asDouble()
    mx0 = data0.get(3).asDouble()
    my0 = data0.get(4).asDouble()
    mz0 = data0.get(5).asDouble()

    fx1 = data1.get(0).asDouble()
    fy1 = data1.get(1).asDouble()
    fz1 = data1.get(2).asDouble()
    mx1 = data1.get(3).asDouble()
    my1 = data1.get(4).asDouble()
    mz1 = data1.get(5).asDouble()

    # ZMP computation
    xzmp0 = (-my0 + e*fx0) / fz0;
    yzmp0 = (mx0 + e*fy0) / fz0;

    xzmp1 = (-my1 + e*fx1) / fz1;
    yzmp1 =( mx1 + e*fy1) / fz1;

    xzmp = (xzmp0 * fz0 + xzmp1 * fz1) / (fz0 + fz1);
    yzmp = (yzmp0 * fz0 + yzmp1 * fz1) / (fz0 + fz1);

    xzmp = xzmp * 1000 # in mm
    yzmp = yzmp * 1000

    #Printing ZMP point
    print "zmp = [" + repr(xzmp) + "," + repr(yzmp) + "] mm"

    plt.plot(yzmp,xzmp,'ko') # axes are changed because of robot axes
    fig.show()
    #Sample time 1ms
    plt.pause(0.001) #delay in seconds
    fig.clf()

p.close()

