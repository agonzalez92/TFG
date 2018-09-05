import yarp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PI = np.pi
alpha = np.pi
n=0
sum_x = 0.0
offs_x = 0.0
sum_y = 0.0
offs_y = 0.0

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
yarp.Network.connect("/jr3/ch0:o","/gui:i")
print("Connection stablished")
# Read the data from de port
data = yarp.Bottle()

fig = plt.figure()

while 1:
	n = n + 1
	# Reading YARP port
	print "Reading..."
	p.read(data) # Forces in Newton and Torques in Newton*meter
	fx = data.get(0).asDouble()
	fy = data.get(1).asDouble()
	fz = data.get(2).asDouble()
	mx = data.get(3).asDouble()
	my = data.get(4).asDouble()
	mz = data.get(5).asDouble()



	if fz == 0:
		x = 0.0
	else:
		x = (-(my/10) / fz )*1000

	if fz == 0:
		y = 0.0
	else:
		y = ((mx/10) / fz )*1000
	
	# OFFSET
        if n >=1 and n < 50:
            	sum_x = x + sum_x;
            	offs_x = sum_x / n;
            	#print "offs = [ ", offs_x_l, " ]"
	if n >=1 and n < 50:
            	sum_y = y + sum_y;
            	offs_y = sum_y / n;
        

        X = x - offs_x
	Y = y - offs_y


	print "zmp = [",X, ",",Y,"]"
	print "n = ",n
	print "sum_x = ",sum_x
	print "offs_x = ",offs_x
	print "sum_y = ",sum_y
	print "offs_y = ",offs_y
	print "fx = ",fx
	print "fy = ",fy
	print "fz = ",fz
	print "mx = ",mx
	print "my = ",my
	print "mz = ",mz

	x1 = [0.0,X]
	y1 = [0.0,Y]

	ax = fig.gca()
	ax.grid()
	plt.plot(y1,x1)
	ax.set_xlim([-100.0,100.0])
	ax.set_ylim([-100.0,100.0])
	fig.show()


	#Sample time 1ms
	plt.pause(0.001) #delay in seconds
	fig.clf()

p.close()


