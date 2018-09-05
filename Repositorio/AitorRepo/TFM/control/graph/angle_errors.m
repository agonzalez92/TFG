%% 2/06/2016
%Commanded angle
angle_ref = [0 -0.5 -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5];
test1AngleLL = [0.15 -0.35 -0.87 -1.31 -1.84 -2.37 -2.81 -3.33 -3.77 -4.39 -4.83];
test1AngleRL = [-0.15 -0.59 -1.12 -1.65 -2.17 -2.79 -3.23 -3.67 -4.2 -4.63 -5.16];
test2AngleLL = [0.15 -0.35 -0.87 -1.23 -1.84 -2.28 -2.81 -3.33 -3.77 -4.39 -4.83];
test2AngleRL = [-0.08 -0.5 -1.03 -1.56 -2 -2.53 -3.05 -3.58 -4.02 -4.55 -5.07];
zmptest1 = [0 0.005 0.012 0.0175 0.024 0.0305 0.037 0.043 0.051 0.06 0.0665];
zmptest2 = [0 0.005 0.01 0.018 0.023 0.034 0.04 0.049 0.052 0.065 0.071];
 

% Relation between commanded angle and angle read from encoders
figure; hold on; box on
plot (angle_ref, test1AngleLL, '-ro', angle_ref, test1AngleRL, '-r*');
plot (angle_ref, test2AngleLL, '-bo', angle_ref, test2AngleRL, '-b*');
xlabel('Angle sent (deg)');
ylabel('Angle read (deg)');
legend('test1LL','test1RL', 'test2LL','test2RL');
title('ommanded angle - angle read from encoders');

figure; hold on; box on;
angle_error = angle_ref - (test1AngleLL-(test1AngleLL - test1AngleRL)/2)
angle_zmp = - (zmptest1 + 0.0018)/ 0.0135;
error_zmp = angle_ref - angle_zmp

plot(angle_error, error_zmp,'b')