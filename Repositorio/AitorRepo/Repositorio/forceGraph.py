import yarp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PI = np.pi
alpha = np.pi

rp = 225/2 # plate radius in mm
rp1 = 140/2 # black plate radius in mm
d = 0.025 # distance in mm between the plate center and the sensor center
def Plate():
    # Big circle
    ang = np.linspace(0,2*PI,360)
    x0 = rp*np.cos(ang)
    y0 = rp*np.sin(ang)

    # Little circle
    ang1 = np.linspace(0,2*PI,360)
    x1 = rp1*np.cos(ang1)
    y1 = rp1*np.sin(ang1)

    x_big = np.concatenate([x0])
    y_big = np.concatenate([y0])
    plt.plot(x_big,y_big,'k')

    x_little = np.concatenate([x1])
    y_little = np.concatenate([y1])
    plt.plot(x_little,y_little,'k')

    #plt.fill(x_big,y_big,'k')
    plt.fill(x_little,y_little,'k')

    return;


# Initialise YARP
yarp.Network.init()
# Create a port
p = yarp.Port()
# Open the port
p.open("/gui:i")
# Connect output and input ports
yarp.Network.connect("/jr3ch2:o", "/gui:i")
print("Connection stablished")
# Read the data from de port
data = yarp.Bottle()

fig = plt.figure()

while 1:
	# Reading YARP port
	print "Reading..."
	p.read(data) # Forces in Newton and Torques in Newton*meter
	fx = data.get(0).asDouble()
	fy = data.get(1).asDouble()
	fz = data.get(2).asDouble()
	mx = data.get(3).asDouble()
	my = data.get(4).asDouble()
	mz = data.get(5).asDouble()



	if fx == 0:
		x = 0.0
	else:
		x = (mx / fx) * 100

	if fy == 0:
		y = 0.0
	else:
		y = (my / fy) * 100



	print "zmp = [",x, ",",y,"]"

	x1 = [0.0,x]
	y1 = [0.0,y]

	ax = fig.gca()
	ax.grid()
	plt.plot(x1,y1)
	ax.set_xlim([-10.0,10.0])
	ax.set_ylim([-10.0,10.0])
	fig.show()


	#Sample time 1ms
	plt.pause(0.001) #delay in seconds
	fig.clf()

p.close()


