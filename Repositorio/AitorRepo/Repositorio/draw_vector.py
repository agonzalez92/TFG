import yarp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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


#while 1:
    #ax = fig.add_subplot(111)
    #ax.grid()
    ##ax.axis('equal')
    #plt.xlim(150,-150) # changed because of robot axes [in mm]
    #plt.ylim(-150,150)
    #ax.set_title('ZMP REPRESENTATION SINGLE WRIST AND PLATE', fontsize=12, fontweight='bold')
    #ax.set_xlabel('Y [mm]')  # changed because of robot axes
    #ax.set_ylabel('X [mm]')  # changed because of robot axes


    # Plotting Plate Area
    #Plate()

    # Reading YARP port
    print "Reading..."
    p.read(data) # Forces in Newton and Torques in Newton*meter

    x = data.get(0).asDouble()
    y = data.get(1).asDouble()



    soa = np.array([[0,0,x,y]])
    X,Y,U,V = zip(*soa)
    ax = fig.gca()
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)
    ax.set_xlim([-600,600])
    ax.set_ylim([-500,500])
    #fig.draw()
    fig.show()

    #Sample time 1ms
    plt.pause(0.001) #delay in seconds
    fig.clf()

p.close()




