data = csvread('/home/teo/Descargas/data.csv');
n = data(:,1);
t = data(:,2);
zmp = data(:,3);
y = data(:,4);
zmp_error = data(:,5);
zmp_ref = data(:,6);
u = data(:,7);
x1 = data(:,8);
x2 = data(:,9);
angle = data(:,10);


%Sole borders
front = (120+55)/1000*ones(size(n));
back = -70/1000*ones(size(n));
left = (140+37.5)/1000*ones(size(n));
right = -(140+37.5)/1000*ones(size(n));


hold on
plot(t,zmp,'g')
plot(t,y,'r')
plot(t,zmp_error,'b')
plot(t,zmp_ref,'k')
plot(t,u,'m')
plot(t,angle,'y')
plot(t,front,'k--',t,back,'k--');

title('K=[13.53, 5.1035], Ki=10, Kp=-0.5, Ku=-1.65')
xlabel('t [s]');
legend('zmp','y{out}','zmp_{error}','zmp_{ref}','u','angle')