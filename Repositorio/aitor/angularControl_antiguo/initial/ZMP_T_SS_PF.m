function [ Yzmp_Teorico] = ZMP_T_SS_PF( ang )

%FUNCTION_ZMP_T_SS_PF Calcula la coordenadas X e Y del ZMP teorico para el
%simple apoyo (SS) en el plano frontal (PF)
% Asumimos m = 50 kg, g = 10 m/s² y L = 0.837 m

m = 50
g = 10
L = 0.837

Xzmp_Teorico = 0;

Yzmp_Teorico = (m * g * L * sin(ang)) / (m * g);


end

