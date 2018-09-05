%% Data (measures in m) - 5/10/2017
angle = [-0.641839087 -1.2837587595 -1.9258396626 -2.5681626797 -3.210808754 -3.8538594246 -4.497396946	-5.1415028572 -5.7862620354 -6.4317564964];
media = [0.0110856693; 0.0221267465; 0.0329036637; 0.0440056769; 0.052838018; 0.0611089384; 0.0716237696; 0.080956724; 0.0956869116; 0.1084994225];


p = polyfit(angle, media', 2);
line = p(1)*angle.^2 + p(2)*angle + p(3); 
pp = polyfit(angle, media', 1);
linep = pp(1)*angle + pp(2);

figure; hold on; box on;
plot(angle, media, 'ko');
plot(angle, media, 'r-')

plot(angle, line,'b');
plot(angle, linep,'g');

title ('Angle - ZMP_F_T relation');
xlabel('angle [deg]');
ylabel('ZMP [m]');
gtext(['ZMP Average = ',num2str(p(1)),'*angle^2 ',num2str(p(2)),'*angle ',num2str(p(3))])
gtext(['ZMP Average = ',num2str(pp(1)),'*angle ',num2str(pp(2))])
legend('Average');


%% Data (measures in mm) - 2/06/2016
% Offset deleted at the beginning
% angles in deg and zmp in milimetres
% angle = [0 -0.5 -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5 -5.5];
% test1 = [-0.8206 9.458172 18.48068 24.52264 32.2026 49.33188 60.02284 62.56476 68.93304 76.47244 90.79408 106.47832];
% test3 = [-0.09604 -0.14688 22.1858 37.80812 6.45656 40.82728 58.9468 62.31672 77.08312 79.3366 86.61288 104.33328];
% test4 = [0.00304 2.55976 15.94772 23.93552 31.9732 39.91068 48.47932 57.58536 64.46004 72.29848 83.3078 93.66324];
% test5 = [-0.31876 8.82712 12.0616 20.44448 32.78604 41.47604 48.41276 56.42836 66.19576 75.09856 81.42444 93.23732];
% test6 = [-0.5078 7.73104 15.87392 23.86504 31.6152 39.63572 46.8846 54.84176 61.886 75.64284 81.07448 88.44352];
% 
% p1 = polyfit(angle, test1, 1);
% line1 = p1(1)*angle + p1(2);
% p3 = polyfit(angle, test3, 1);
% line3 = p3(1)*angle + p3(2);
% p4 = polyfit(angle, test4, 1);
% line4 = p4(1)*angle + p4(2);
% p5 = polyfit(angle, test5, 1);
% line5 = p5(1)*angle + p5(2);
% p6 = polyfit(angle, test6, 1);
% line6 = p6(1)*angle + p6(2);
% 
% figure; hold on; box on;
% 
% plot(angle, test1, 'bo');
% plot(angle, test3, 'ro');
% plot(angle, test4, 'go');
% plot(angle, test5, 'yo');
% plot(angle, test6, 'mo');
% 
% plot(angle, line1,'r');
% plot(angle, line3,'r');
% plot(angle, line4,'r');
% plot(angle, line5,'r');
% plot(angle, line6,'r');
% 
% xlabel('angle [deg]');
% ylabel('ZMP [mm]');
% gtext(['Test 1: zmp = ',num2str(p1(1)),'*angle +',num2str(p1(2))])
% gtext(['Test 2: zmp = ',num2str(p3(1)),'*angle +',num2str(p3(2))])
% gtext(['Test 3: zmp = ',num2str(p4(1)),'*angle +',num2str(p4(2))])
% gtext(['Test 4: zmp = ',num2str(p5(1)),'*angle +',num2str(p5(2))])
% gtext(['Test 5: zmp = ',num2str(p6(1)),'*angle +',num2str(p6(2))])
% legend('ZMP F/T Test1','ZMP F/T Test2', 'ZMP F/T Test3', 'ZMP F/T Test4', 'ZMP F/T Test5');
% 

