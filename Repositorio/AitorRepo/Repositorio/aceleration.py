import yarp
import matplotlib.pyplot as plt


# Initialise YARP
yarp.Network.init()
# Create a port
p = yarp.Port()
# Open the port
p.open("/gui:i")
# Connect output and input ports
yarp.Network.connect("/inertial", "/gui:i")
print("Connection stablished")
# Read the data from de port
data = yarp.Bottle()

fig = plt.figure()

while 1:
	# Reading YARP port
	print "Reading..."
	p.read(data) # Forces in Newton and Torques in Newton*meter
	acx = data.get(3).asfloat()
	acy = data.get(4).asfloat()
	print "Aceleration X: ",acx,u" m/s\xb2"
	print "Aceleration Y: ",acy,u" m/s\xb2"
	ax1 = [0.0,acy]
	ay1 = [0.0,acx]
	ax = fig.gca()
	ax.grid()
	plt.plot(ax1,ay1)
	ax.set_xlim([-10.0,10.0])
	ax.set_ylim([-10.0,10.0])
	fig.show()

	#Sample time 1ms
	plt.pause(0.001) #delay in seconds
	fig.clf()

p.close()


