function [ sys_FT ] = paso3_calculo_ft ( ZMP_ref )

%FUNCTION calculo_ka, Calcula los distontos valores de ka, para cada ZMP_ref
% Asumimos m = 62.589 kg, g = 9.81 m/s² y L = 0.8927 m

m = 62.589;
g = 9.81;
L = 0.8927;
chi = 1;

% ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-g/L^2) / ((ka-g)./L)
ANG_ref = -(asin(ZMP_ref/L) * 180/pi) ;
ka = ((ANG_ref*-1*g/L)/ZMP_ref)+g;

% Beta = (ka-g)./L = Wn^2
Wn = sqrt((ka-g)/L);

% Alpha = ba./(m*L) = 2*chi*Wn
ba = 2*chi*Wn*m*L;

K = -g/L^2;
Num = K;

Alpha = ba/(m*L);
Beta = (ka-g)/L;
Den = [1 Alpha Beta];

sys_FT = tf (Num,Den);
%impulse(sys_FT)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',ANG_ref);
%[yout,x,t]=
step(sys_FT,opt);
%roots(Den)
end