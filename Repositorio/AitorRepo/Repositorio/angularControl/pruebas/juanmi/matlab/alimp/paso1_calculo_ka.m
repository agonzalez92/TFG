function [ ka ] = paso1_calculo_ka ( ZMP_ref )

%FUNCTION calculo_ka, Calcula los distontos valores de ka, para cada ZMP_ref
% Asumimos m = 62.589 kg, g = 9.81 m/s� y L = 0.8927 m

m = 62.589;
g = 9.81;
L = 0.8927;
%chi = 0.6;
% ka_adj=9.9725; %ref=0.09
% ANG_ref_adj = -6.0863; %ref=0.09
ka_adj=9.9650; %ref=0.06
ANG_ref_adj = -4.24305; %ref=0.06

a = 0.834;
b = 1.024;
c = -0.0004;

% ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-g/L^2) / ((ka-g)./L)
%Calculo de Ka
% ANG_ref2 = -(asin(ZMP_ref./L) .* 180./pi);
% ZMP_ref_adj = a.*ZMP_ref.*ZMP_ref + b*ZMP_ref + c
% ka = ((ZMP_ref_adj.*-1*g./L)./ANG_ref2)+g;

%Calculo de ZMP_ref_adj a partir de Ka experimental
%ANG_ref_adj = (ZMP_ref_adj*(-g))/ (L*(ka_adj-g)) 
% ZMP_ref_adj=(ka_adj-g)*ANG_ref_adj*L./(-g);

%Calculo de Ka - juanmi
% ANG_ref_adj = (ZMP_ref*(-g))/ (L*(ka_adj-g))
% ka = ((ZMP_ref.*-1*g./L)./ANG_ref_adj)+g;

%Calculo de ang_ref - juanmi
ka = 0.25 * ZMP_ref + 9.95
ang_ref = (ZMP_ref*(-g))/ (L*(ka-g))

angle_ref_a = ZMP_ref*(-g)
angle_ref_b = (ka-g)
angle_ref_c = L
angle_ref_d = L*angle_ref_b
angle_ref_1 = angle_ref_a/angle_ref_d

end