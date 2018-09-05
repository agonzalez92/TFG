"""
Demo of a PathPatch object.
"""
import matplotlib.pyplot as plt
import numpy as np

PI = np.pi
d = 146-17.52
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

    x_red = np.concatenate([-x0,-x1,-x2,-x3])
    y_red = np.concatenate([y0,y1,y2,y3])
    plt.plot(x_red,y_red,'k')

    x_yellow = np.concatenate([-x0,-x1,-x2,-x3])*2/3 - (d-(d*2/3))
    y_yellow = np.concatenate([y0,y1,y2,y3])*2/3 + (h-(h*2/3))
    plt.plot(x_yellow,y_yellow,'k')

    x_green = np.concatenate([-x0,-x1,-x2,-x3])*1/3 - (d-(d*1/3))
    y_green = np.concatenate([y0,y1,y2,y3])*1/3 + (h-(h*1/3))
    plt.plot(x_green,y_green,'k')

    plt.fill(x_red,y_red,'r')
    plt.fill(x_yellow,y_yellow,'y')
    plt.fill(x_green,y_green,'g')
    return;





fig = plt.figure()

ax = fig.add_subplot(111)
ax.grid()
ax.axis('equal')
ax.set_title('ZMP REPRESENTATION IN SINGLE SUPPORT', fontsize=12, fontweight='bold')
ax.set_xlabel('x [mm]')
ax.set_ylabel('y [mm]')

RightFoot()
LeftFoot()

ax.plot(0,0,'k',linewidth=2.0)

plt.show()
