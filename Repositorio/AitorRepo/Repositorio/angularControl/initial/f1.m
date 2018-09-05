function [ L ] = f1( Xzmp,fx,fz )
%UNTITLED Summary of this function goes here
%FUNCION_L Calcula la longitud del pendulo invertido en funcion de los
%parametros del Xzmp de Loli
% Asumimos m = 50 kg, g = 10 m/s² y e = 0.194 m
m = 50 
g = 10
e = 0.0194

L = ((Xzmp'* 1000) * fz + e * fx) / (m * g);


end

