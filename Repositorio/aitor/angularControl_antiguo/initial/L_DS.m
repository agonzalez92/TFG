function [ L ] = L_DS( Xzmp, fx0, fx1, fz0, fz1)
%FUNCION_L_DS Calcula la longitud del pendulo invertido en funcion de los
%parametros del Xzmp de Loli para doble apoyo
% Asumimos m = 58 kg, g = 9.81 m/s² y e = 0.0194 m
m = 58 
g = 9.81
e = 0.03225

L = - (((Xzmp' / 1000) * (fz0 + fz1)) * 10 + 10 * e * (fx0 + fx1)) / (2 * m * g);


end

