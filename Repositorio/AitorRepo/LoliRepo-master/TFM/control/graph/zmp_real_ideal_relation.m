% Data (measures in metres)
zmptest1 = [0 0.0015 0.0043 0.0085 0.013 0.017 0.021 0.024 0.027 0.033 0.0365];
ref = [0 0.005 0.01 0.015 0.02 0.025 0.03 0.035 0.04 0.045 0.05];
angle = -asin(zmptest1 / 0.8927) *180/pi;
angle_ref = -asin(ref / 0.8927) *180/pi;
zmp_ideal =  ref;

% Relation between the ZMP commanded and the ZMP obtained from sensors
figure; hold on;
plot(ref, zmptest1 ,'bo');
plot(ref, zmp_ideal, 'm');
xlabel('ZMP referencia');
ylabel('ZMP F/T');
legend('ZMP real', 'ZMP ideal')

% Lines above have a linear error increment
%figure;
error =  ref - (0.76*ref-0.002);
plot(ref, error)
xlabel('ZMP ref')
ylabel('error')

% Relation between angle commanded (ZMP/L)*180/pi and ZMP read
figure; hold on;
%plot(angle_ref, ref, 'r');
plot(angle_ref, zmptest1, 'b')
xlabel('angle sent (deg)');
ylabel('zmp read (mm)');


%Comparation angle-ZMP
figure; hold on;
plot(angle_ref, zmptest1,'bo');
zmp1 = -0.01558 * angle_ref;
plot(angle_ref, zmp1,'r')
xlabel('angle(deg)');
ylabel('ZMP (m)');
legend('no offset', 'with offset');


%Aproximacion a 1/x
cociente = error ./ ref;
y = (0.004 ./ (1.5*ref)) + 0.23 ;
figure; hold on
plot(ref, cociente,'bo');
plot(ref, y,'r-');
legend('cociente', 'ajuste');
gtext('y = (0.004/(1.5*ref)) + 0.23')
xlabel('ref');
ylabel('error/ref');

ref = 0.02;
theta = (ref/0.0156) + (0.004/(1.5*ref)+0.23)*ref/0.0156


%Aproximacion a exponencial y = c*e^ax
cociente = error ./ ref;
p = polyfit(ref,log(cociente),1);
a = p(1);
c = exp(p(2));
fprintf('exponente a= %2.3f\n',a);
fprintf('coeficiente c = %3.3f\n',c);

y =  c * exp(a*ref);

y2 =  c * exp(-100*ref)+0.28;

plot(ref,y,'r',ref,y2,'g',ref, cociente, 'bo')
xlabel('ref');
ylabel('error/ref');
legend('y = 0.51 * e^-14.792*ref', 'y = (0.51 * e^-100*ref )+0.28' )

ref = 0.07;
o = -((ref/0.0135) + ((0.5186*exp(-100*ref))))

ref= 0.01;
o = -(ref/0.0156) - ((0.5186*exp(-14.79*ref)-12*ref))

% Comparation between angle error and zmp error.
error =  ref - (0.76*ref-0.002);
angle_error = +(0.76*ref-0.002)/0.0156;

plot(error, angle_error, 'bo')
xlabel('zmp')
ylabel('|theta|');

%% 7/06/2016
clear;
zmp_ref = [0:0.005:0.1]; % Reference setpoint
angle = -zmp_ref/0.0135; % Commanded angle
% ZMP F/T measuerment
zmptest1 = [0 0.0032 0.0075 0.0115 0.0152 0.0202 0.0247 0.0327 0.037 ...
    0.043 0.0525 0.0575 0.0637 0.071 0.0765 0.0833 0.09 0.102 0.105 ...
    0.1185 0.1205];
zmp_ideal = zmp_ref;
% Angles of Right and Left Leg. Encoders measurements.
angleLL = [0.15 -0.263 -0.615 -0.966 -1.31 -1.757 -2.108 -2.46 -2.811 ...
    -3.251 -3.602 -3.95 -4.39 -4.74 -5.09 -5.536 -5.88 -6.239 -6.678 ...
    -6.942 -7.293];
angleRL = [-0.15 -0.509 -0.949 -1.3 -1.652 -2 -2.35 -2.794 -3.145 ...
    -3.585 -3.936 -4.288 -4.63 -5.079 -5.430 -5.869 -6.13 -6.429 -6.836 ...
    -7.188 -8.246];

% Second order fitting to zmp data
p1 = polyfit(zmp_ref, zmptest1, 2);
zmp_linearized = p1(1)*zmp_ref.^2 + p1(2)*zmp_ref + p1(3); 

% Second order fitting to error
error = zmp_ref - zmp_linearized;
p2 = polyfit(zmp_ref, error, 2);
error_line = p2(1)*zmp_ref.^2 + p2(2)*zmp_ref + p2(3); 

% Angle that has to be commanded instead of the angle above
angle2 = angle + (p2(1)*zmp_ref.^2 + p2(2)*zmp_ref)/0.0135
angle3 = angle + (p2(1)*zmp_ref.^2 + 0.25*zmp_ref)/0.0135
 
figure; hold on; box on; grid on;
plot(zmp_ref, zmp_ideal, 'k', zmp_ref, zmptest1, 'bo');
plot(zmp_ref, zmp_linearized, 'r');
plot(zmp_ref, error,'g')
title('Comparation real ZMP - ideal ZMP when angle = ZMP / 0.0135')
xlabel('Reference ZMP [mm]')
ylabel('[mm]')
legend('Ideal ZMP', 'ZMP F/T','linearized ZMP F/T','error')
text(0.01,-0.02,['ZMP = ',num2str(p1(1)),'*ZMP_{ref}^2 +',num2str(p1(2)), ...
    '*ZMP_{ref} +', num2str(p1(3))])
text(0.01,-0.03,['error = ',num2str(p2(1)),'*ZMP_{ref}^2 +',num2str(p2(2)), ...
    '*ZMP_{ref} +', num2str(p2(3))])

% Comparation between commanded angle and RL and LL encoder data..
.

figure; hold on; box on; grid on;
plot(angle, angle, 'k', angle, angleLL, 'ob-', angle, angleRL, 'or-');

%% 8/06/2016
% test with the cuadratic relation

zmp_ref = [0 0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.08 0.09 0.1];
% For test 1 angle = -(zmp_ref -4.3948*zmp_ref.^2 + 0.19(2)*zmp_ref))/0.0135
zmptest1 = [0 0.0085 0.018 0.029 0.038 0.0475 0.055 0.07 0.076 0.089 0.085];
% For test 2 angle = -(zmp_ref -4.3948*zmp_ref.^2 + 0.25*zmp_ref))/0.0135
zmptest2 = [0 0.0115 0.021 0.0317 0.0387 0.051 0.062 0.0715 0.079 0.091 0.095];
zmp_ideal = zmp_ref;

figure; hold on; box on; grid on;
plot(zmp_ref, zmp_ideal, 'k', zmp_ref, zmptest1, 'bo', zmp_ref, zmptest2, 'mo');
title('ZMP read (added cuadratic error)')
xlabel('Reference ZMP [mm]')
ylabel('ZMP read [mm]')
legend('ideal', 'test1','test2')