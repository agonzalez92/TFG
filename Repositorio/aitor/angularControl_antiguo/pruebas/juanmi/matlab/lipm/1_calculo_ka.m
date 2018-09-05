function [ ka ] = calculo_ka ( ZMP_ref )

%FUNCTION calculo_ka, Calcula los distontos valores de ka, para cada ZMP_ref
% Asumimos m = 62.589 kg, g = 9.81 m/s² y L = 0.8927 m

m = 62.589
g = 9.81
L = 0.8927
chi = 0.6

% ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-g/L^2) / ((ka-g)./L)
ANG_ref = -(asin(ZMP_ref./L) .* 180./pi) 
ka = ((ANG_ref.*-1*g./L)./ZMP_ref)+g

end