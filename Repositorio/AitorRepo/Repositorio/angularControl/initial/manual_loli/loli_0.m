function [ ZMP_RF ] = loli_0 ( my0, fx0, fz0 )

e=0.03225
ZMP_RF = -((( my0./10) + e.* fx0)*1000) ./ fz0

end

