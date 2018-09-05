%% Data (measures in mm) - 19/05/2016
angle = [0 -0.5 -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5];
test1 = [-14.5; -9; -3.5; 0; 8.5; 17.5; 25; 33.5; 47.5; 53; 60]/1000;
test2 = [-14.5; -8.6; -3.7; -2.5; 8.6; 17; 24.5; 32.5; 42.5; 51; 56.5]/1000;
test3 = [-16.5; -12; -6.5; 0.5; 8; 16.5; 24.5; 33; 41; 50.5; 57.5]/1000;
test4 = [-15.5; -10.7; -4.5; 1.5; 8.2; 15.5; 23.5; 33; 40.5; 47.8; 54.5]/1000;
test5 = [-13.5; -8.4; -2.8; 3.9; 10.2; 17.6; 26.1; 34.8; 42.6; 52.1; 60.5]/1000;
test = [test1 test2 test3 test4 test5];

%Valor medio para cada angulo
Vm = mean (test,2);

hold on;
plot(angle, test1, 'bx');
plot(angle, test2, 'rx');
plot(angle, test3, 'gx');
plot(angle, test4, 'yx');
plot(angle, test5, 'mx');
plot(angle, Vm, 'ko')
plot(angle, Vm, 'k-')

title ('Angle - ZMP relation');
xlabel('angle [deg]');
ylabel('ZMP [m]');
legend('test1','test2','test3','test4','test5', 'Average');


%% Data (measures in mm) - 2/06/2016
% Offset deleted at the beginning
% angles in deg and zmp in metres
angle = [0 -0.5 -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5];
test1 = [0 0.005 0.012 0.0175 0.024 0.0305 0.037 0.043 0.051 0.06 0.0665];
test2 = [0 0.005 0.01 0.018 0.023 0.034 0.04 0.049 0.052 0.065 0.071];
test3 = [0 0.004 0.008 0.014 0.021 0.026 0.036 0.04 0.047 0.055 0.060];
p = polyfit(angle, test1, 1);
line = p(1)*angle + p(2);
p2 = polyfit(angle, test2, 1);
line2 = p2(1)*angle + p2(2);
p3 = polyfit(angle, test3, 1);
line3 = p3(1)*angle + p3(2);

figure; hold on; box on;
plot(angle, test1, 'bo');
plot(angle, test2, 'go');
plot(angle, test3, 'mo');
plot(angle, line,'r');
plot(angle, line2,'r');
plot(angle, line3,'r');
xlabel('angle [deg]');
ylabel('ZMP [mm]');
gtext(['Test 1: zmp = ',num2str(p(1)),'*angle +',num2str(p(2))])
gtext(['Test 2: zmp = ',num2str(p2(1)),'*angle +',num2str(p2(2))])
gtext(['Test 3: zmp = ',num2str(p3(1)),'*angle +',num2str(p3(2))])
legend('ZMP F/T Test1','ZMP F/T Test2', 'ZMP F/T Test3');


