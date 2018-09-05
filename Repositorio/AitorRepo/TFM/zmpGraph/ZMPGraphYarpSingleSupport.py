import yarp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PI = np.pi
d = 70+37.5
h = 0
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


    x_red = np.concatenate([x0,x1,x2,x3])
    y_red = np.concatenate([y0,y1,y2,y3])
    plt.plot(x_red,y_red,'k')

    x_yellow = np.concatenate([x0,x1,x2,x3])*2/3 + (d-(d*2/3))
    y_yellow = np.concatenate([y0,y1,y2,y3])*2/3 + (h-(h*2/3))
    plt.plot(x_yellow,y_yellow,'k')

    x_green = np.concatenate([x0,x1,x2,x3])*1/3 + (d-(d*1/3))
    y_green = np.concatenate([y0,y1,y2,y3])*1/3 + (h-(h*1/3))
    plt.plot(x_green,y_green,'k')

    plt.fill(x_red,y_red,'r')
    plt.fill(x_yellow,y_yellow,'y')
    plt.fill(x_green,y_green,'g')

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
p = yarp.Port()
# Open the port
p.open("/gui:i")
# Connect output and input ports
yarp.Network.connect("/jr3:o", "/gui:i")
# Read the data from de port
data = yarp.Bottle()

fig = plt.figure()

while 1:
    ax = fig.add_subplot(111)
    ax.grid()
    #ax.axis('equal')
    plt.xlim(200,-200)
    plt.ylim(-200,200)
    ax.set_title('ZMP REPRESENTATION IN SINGLE SUPPORT', fontsize=12, fontweight='bold')
    ax.set_xlabel('y [mm]')  # changed because of robot axes
    ax.set_ylabel('x [mm]')  # changed because of robot axes

    # Plotting ZMP Areas
    RightFoot()
    LeftFoot()

    # Reading YARP port
    print "Reading..."
    p.read(data)
    x = data.get(0).asDouble() * 1000 #in milimeters
    y = data.get(1).asDouble() * 1000 #in milimeters

    #Printing ZMP point
    print "zmp = [" + repr(x) + "," + repr(y) + "]"

    plt.plot(y,x,'ko') # axes are changed because of robot axes
    o = 3
    plt.plot(y+o,x+o,'ko')
    plt.plot(y,x+o,'ko')
    plt.plot(y+o,x,'ko')
    plt.plot(y-o,x-o,'ko')
    plt.plot(y,x-o,'ko')
    plt.plot(y-o,x,'ko')
    o1=5
    plt.plot(y+o1,x+o1,'ko')
    plt.plot(y,x+o1,'ko')
    plt.plot(y+o1,x,'ko')
    plt.plot(y-o1,x-o1,'ko')
    plt.plot(y,x-o1,'ko')
    plt.plot(y-o1,x,'ko')
    fig.show()

    #Sample time 1ms
    plt.pause(0.001) #delay in seconds
    fig.clf()

p.close()

    #Status API Training Shop Blog About Pricing 


