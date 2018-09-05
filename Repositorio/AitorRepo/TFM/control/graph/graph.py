import numpy as np
import matplotlib.pyplot as plt
import csv

data = np.loadtxt('/home/teo/data.csv', delimiter=",")
n = data[:,0]
t = data[:,1]
zmp = data[:,2]
y = data[:,3]
error = data[:,4]
ref = data[:,5]
u = data[:,6]
x1 = data[:,7]
x2 = data [:,8]
angle = data[:,9]

# Sole borders
front = ((120+55)/1000)*np.ones(n.size);
back = -70/1000*np.ones(n.size);
#left = (140+37.5)/1000*np.ones(size(n));
#right = -(140+37.5)/1000*np.ones(size(n));

fig = plt.figure()

plt.plot(t, zmp, 'g-', label= 'zmp')
plt.plot(t, y, 'r-', label = 'zmp_model')
plt.plot(t, error, 'b-', label = 'error')
plt.plot(t,ref,'k-', label = 'ref')
plt.plot(t,u,'m-', label = 'u')
plt.plot(t,x1,'y')
plt.plot(t,x2,'c')
plt.plot(t,angle,'y-', label = 'angle')
plt.plot(t,front,'k--',t,back,'k--');

plt.title('K=[12.55, 4.91], Ki=10, Kp=-1.5, Ku=-2.05')
plt.xlabel('t [s]')

plt.show()


