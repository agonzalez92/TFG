function [ Xzmp_Teorico ] = ZMP_T_DS_PS( ang )

%FUNCTION_ZMP_T_SS_PS Calcula la coordenadas X e Y del ZMP teorico para el
%doble apoyo (DS) en el plano sagital (PS)
% Asumimos m = 50 kg, g = 10 m/s² y L = 0.837 m

m = 50
g = 10
L = 837

Xzmp_Teorico =  (m * g * L * sin(ang)) / (m * g) ;

Yzmp_Teorico = 0;

end

