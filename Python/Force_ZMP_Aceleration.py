import yarp
import numpy as np
import matplotlib.pyplot as plt

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
    y1 = h + np.linspace(1,121,121) - 1

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
    y1 = h + np.linspace(1,121,121) - 1

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
p0 = yarp.Port()
p1 = yarp.Port()
# Open the port
p.open("/gui:i")
p0.open("/gui0:i")
p1.open("/gui1:i")
# Connect output and input ports
yarp.Network.connect("/inertial", "/gui:i")
yarp.Network.connect("/jr3/ch0:o", "/gui0:i") # Left foot sensor
yarp.Network.connect("/jr3/ch1:o", "/gui1:i") # Right foot sensor
print("Connection stablished")
# Read the data from de port
data = yarp.Bottle()data0 = yarp.Bottle()
data1 = yarp.Bottle()

fig = plt.figure()

while 1:
    	ax = fig.add_subplot(111)
    	ax.grid()
    	ax = fig.gca()
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
	# Forces in Newton and Torques in Newton*meter
	p.read(data) 
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

	acx = data.get(3).asDouble()
	acy = data.get(4).asDouble()

	# ZMP computation
    	if fz0 == 0:
		xzmp0 = 0
		yzmp0 = 0
    	else:
    		xzmp0 = (-my0 + e*fx0) / fz0;
    		yzmp0 = (mx0 + e*fy0) / fz0;

   	if fz1 == 0:
		xzmp1 = 0
		yzmp1 = 0
    	else:
		xzmp1 = (-my1 + e*fx1) / fz1;
		yzmp1 =( mx1 + e*fy1) / fz1;

    	if fz0 == 0 and fz1 == 0:
		xzmp = 0
		yzmp = 0
    	else:
		xzmp = (xzmp0 * fz0 + xzmp1 * fz1) / (fz0 + fz1);
    		yzmp = (yzmp0 * fz0 + yzmp1 * fz1) / (fz0 + fz1);

    	xzmp = xzmp * 1000 # in mm
    	yzmp = yzmp * 1000

    	#Fuerza pie izquierdo
    	if fx0 == 0:
		x0 = 0.0
    	else:
		x0 = (my0 / fx0) * 100

    	if fy0 == 0:
		y0 = 0.0
    	else:
		y0 = (mx0 / fy0) * 100

    	x00 = [128.5,x0]
    	y00 = [0.0,y0]


    	#Fuerza pie derecho
    	if fx1 == 0:
		x1 = 0.0
    	else:
		x1 = (my1 / fx1) * 100

    	if fy1 == 0:
		y1 = 0.0
    	else:
		y1 = (mx1 / fy1) * 100

    	x11 = [-128.5,x1]
    	y11 = [0.0,y1]

    	# Vector from left foot sensor to ZMP
    	xzmp0 = [128.5,yzmp]   
    	yzmp0 = [0.0,xzmp]

    	# Vector from right foot sensor to ZMP
    	xzmp1 = [-128.5,yzmp]  
    	yzmp1 = [0.0,xzmp]
    	
	# Aceleration
	ax1 = [0.0,acy]
	ay1 = [0.0,acx]
    	# axes are changed because of robot axes

    	#Printing ZMP point
    	print "zmp = [" + repr(xzmp) + "," + repr(yzmp) + "] mm"
   	print "coord. rigth foot = [ ",x0,",",y0," ]"
    	print "coord. left foot = [ ",x1,",",y1," ]"
	print "Aceleration X: ",acx,u" m/s\xb2"
	print "Aceleration Y: ",acy,u" m/s\xb2"

	
	
	plt.plot(ax1,ay1)
	plt.plot(yzmp,xzmp,'ko') # axes are changed because of robot axes
    	plt.plot(x00,y00)
    	plt.plot(x11,y11)
    	plt.plot(xzmp0,yzmp0,'r')
    	plt.plot(xzmp1,yzmp1,'r')
	fig.show()

	#Sample time 1ms
	plt.pause(0.001) #delay in seconds
	fig.clf()

p.close()


