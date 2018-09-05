function [ Ba ] = paso2_calculo_Ba ( ZMP_ref )

%FUNCTION calculo_ka, Calcula los distontos valores de ka, para cada ZMP_ref
% Asumimos m = 62.589 kg, g = 9.81 m/s� y L = 0.8927 m

m = 62.589
g = 9.81
L = 0.8927
chi = 0.6

a = 0.834;
b = 1.024;
c = -0.0004;

% ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-g/L^2) / ((ka-g)./L)
ANG_ref = -(asin(ZMP_ref./L) .* 180./pi);
ZMP_ft = a.*ZMP_ref.*ZMP_ref + b*ZMP_ref + c;
ka = ((ZMP_ft.*-1*g./L)./ANG_ref)+g;

% Beta = (ka-g)./L = Wn^2
Wn = sqrt((ka-g)./L);

% Alpha = ba./(m*L) = 2*chi*Wn
Ba = 2*chi*Wn*m*L

end