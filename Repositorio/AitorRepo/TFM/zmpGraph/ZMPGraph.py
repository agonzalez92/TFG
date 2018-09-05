import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
plt.hold(True)

#No-stability area vertex
p11=[-4,-4]
p12=[2,-4]
p13=[2,6]
p14=[-4,6]

#Unknown stability area vertex
p21=[-2.5,-2]
p22=[1,-2]
p23=[1,4]
p24=[-2.5,4]

#Stability area vertex
p31=[-1.5,-1]
p32=[0.5,-1]
p33=[0.5,2.5]
p34=[-1.5,2.5]

#No-stability area (RED)
x1=[p11[0],p12[0],p13[0],p14[0],p11[0]]
y1=[p11[1],p12[1],p13[1],p14[1],p11[1]]

#Unknown stability area (YELLOW)
x2=[p21[0],p22[0],p23[0],p24[0],p21[0]]
y2=[p21[1],p22[1],p23[1],p24[1],p21[1]]

#Stability area (GREEN)
x3=[p31[0],p32[0],p33[0],p34[0],p31[0]]
y3=[p31[1],p32[1],p33[1],p34[1],p31[1]]

plt.axis([-10,10,-10,10])
plt.plot(x1,y1,'k',x2,y2,'k',x3,y3,'k',linewidth=2.0)
plt.fill(x1,y1,'r')
plt.fill(x2,y2,'y')
plt.fill(x3,y3,'g')



def update_zmp(i,zmp,point):
	point.set_data(zmp[i,0], zmp[i,1])
	return point,


zmp=np.matrix([[0,0],[2,1],[1,3],[0,0],[8,2],[5,4]])
#zmp =  np.random.rand(2, 25)
#for i in [0,1,2,3,4,5]:
#	plt.plot(zmp[i,0],zmp[i,1],'ro')
#	plt.show()
point, = plt.plot([],[],'ko')


ani = animation.FuncAnimation(fig, update_zmp,6,fargs=(zmp, point) , interval=1000)

plt.show()






