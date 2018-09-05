function [ L ] = L_SS( Xzmp,fx,fz )
%UNTITLED Summary of this function goes here
%FUNCION_L_SS Calcula la longitud del pendulo invertido en funcion de los
%parametros del Xzmp de Loli para simple apoyo
% Asumimos m = 58 kg, g = 9.81 m/s² y e = 0.0194 m
m = 58 
g = 9.81
e = 0.03225

L = - (((Xzmp'/ 1000) * fz) * 10 + 10 * e * fx) / (m * g);


end

