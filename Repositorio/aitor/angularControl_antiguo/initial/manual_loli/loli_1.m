function [ ZMP_LF ] = loli_1 ( my1, fx1, fz1 )

e=0.03225
ZMP_LF = -((( my1./10) + e.* fx1)*1000) ./ fz1

end

