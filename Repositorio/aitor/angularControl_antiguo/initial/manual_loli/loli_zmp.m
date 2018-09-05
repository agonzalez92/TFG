function [ ZMP_DS ] = loli_zmp ( ZMP_RF, ZMP_LF, fz0, fz1 )

ZMP_DS = (ZMP_RF .* fz0 + ZMP_LF .* fz1) ./ (fz0 + fz1); 

end

