% calculo de los valores medios tanto del ZMP_ref como del ZMP_FT. El objetivo es 
% observa la relacion que hay entre estos dos ZMP's y detectar cual es error que
% se esta comentiendo.

% primer paso: dimensionar el tamaño de los vectores a utilizar
iter=[1:1:size(XzmpLoli2)];

% segundo paso: calcular la media del ZMP_REF
h=1;
i=1
long=size(iter,2)
media_ref=0;
for i=250:500:long-321
    for n=i:1:i+250
      media_ref = media_ref+eval_xzmp_ref_mm(n);
    end
    media_ref=media_ref/250
    ZMP_REF(h)=media_ref
    h=h+1
    media_ref=0
end

ZMP_REF=ZMP_REF';

% tercer paso: calcular la media del ZMP_FT
j=1
k=1
media_ref=0;
for k=250:500:long-342
    for n=k:1:k+250
      media_ref = media_ref+XzmpLoli2(n);
    end
    media_ref=media_ref/250
    ZMP_FT(j)=media_ref
    j=j+1
    media_ref=0
end

ZMP_FT=ZMP_FT';

% cuarto paso: plotear la grafica, mostrando la relacion y la funcion
% resultante del ZMP_FT.
step=[0:1:j-2]

p = polyfit(ZMP_REF, ZMP_FT, 2);
line = p(1)*ZMP_REF.^2 + p(2)*ZMP_REF + p(3);
pp = polyfit(ZMP_REF, ZMP_FT, 1);
linep = pp(1)*ZMP_REF + pp(2);

figure; hold on; box on;

plot(step, ZMP_REF, 'b-')
plot(step, ZMP_FT, 'ro')
plot(step, ZMP_FT, 'g-')

% plot(ZMP_REF, line,'r');
% plot(ZMP_REF, linep,'g');

% title ('ZMPs relation');
xlabel('Iterations');
ylabel('ZMP [mm]');
% gtext(['FT VS REF = ',num2str(p(1)),'*ZMP_R_E_F^2 ',num2str(p(2)),'*ZMP_R_E_F ',num2str(p(3))])
% gtext(['FT VS REF = ',num2str(pp(1)),'*ZMP_R_E_F ',num2str(pp(2))])
legend('ZMP_r_e_f','ZMP_F_T');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
% test = [test1 test3 test4 test5 test6];
% 
% %Valor medio para cada angulo
% %
% Vm = mean (test,2);
% 
% p = polyfit(angle, Vm', 2);
% line = p(1)*angle.^2 + p(2)*angle + p(3); 
% pp = polyfit(angle, Vm', 1);
% linep = pp(1)*angle + pp(2);
% 
% figure; hold on; box on;
% plot(angle, test1, 'bo');
% plot(angle, test3, 'ro');
% plot(angle, test4, 'go');
% plot(angle, test5, 'yo');
% plot(angle, test6, 'mo');
% plot(angle, Vm, 'kx')
% plot(angle, Vm, 'k-')
% 
% plot(angle, line,'r');
% plot(angle, linep,'g');
% 
% title ('Angle - ZMP relation');
% xlabel('angle [deg]');
% ylabel('ZMP [mm]');
% gtext(['ZMP Average = ',num2str(p(1)),'*angle^2 ',num2str(p(2)),'*angle ',num2str(p(3))])
% gtext(['ZMP Average = ',num2str(pp(1)),'*angle ',num2str(pp(2))])
% legend('test1','test2','test3','test4','test5', 'Average');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% step=[0:1:j-2]
% plot(step, ZMP_REF, 'r');
% hold on;
% plot(step, ZMP_FT, 'bo');
% hold off;