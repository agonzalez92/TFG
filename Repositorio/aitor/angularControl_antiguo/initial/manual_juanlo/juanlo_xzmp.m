function [ ZMP_JUANLO] = juanlo_xzmp ( my0, fx0, fz0 )

Zcom = 1.036602
radianes = angulo .* pi ./ 180
seno = sin (radianes)
coseno = cos (radianes)
ZMP_JUANLO = Xzmp_juanlo - ((Zcom * (x_acc_robot*seno - z_acc_robot*coseno))/

end

