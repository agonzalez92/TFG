m  = 62.589; %System mass
g = 9.81; %Gravity constant
%l = (92+330+299.897+123.005+191.7)/1000; %Pendulum longitude
l = 0.8927;
k = 200; %Stiffness
T = 0.03; %Sample Time

a = (k - m*g*l) / (m*l^2);
b = k / (m*l^2);

%% Space state matrices
A = [0 1; -a 0]; 
B = [0; 1];
C = [-(k/(m*g))*b 0];
D = k/(m*g);
sys = ss(A,B,C,D);
sys_d = c2d(sys,T,'zoh');

%polos
poles = eig(A);

% %controlabilidad
% co =ctrb(A,B);
% unco = length(A)-rank(co);
% 
% co_d = ctrb(sys_d);
% ob_d = obsv(sys_d);
% controllability_d = rank(co_d);
% observability_d = rank(ob_d);

% %% Transfer function
% [num,den] = ss2tf(A,B,C,D);
% G_s = tf(num,den);
% t = 0:0.001:10;
% u = 0.1*ones(size(t));
% lsim(G_s,u,t)
% 
% % Add a pole in s = 0
% den = [den 0];
% G2_s = tf(num,den);
% lsim(G_s,u,t)
% 
% % Closing the loop
% H_s = feedback(G2_s,-1);
% lsim(H_s,u,t);


%% DISCRETE LQR DESIGN
Q = C'*C;
%Q(1,1) = 1000;
R = 1;

[K] = dlqr(sys_d.a, sys_d.b, Q, R);

Ac = (sys_d.a-sys_d.b*K);
Bc = sys_d.b;
Cc = sys_d.c;
Dc = sys_d.d;

sys_cl = ss(Ac,Bc,Cc,Dc,T);

t = 0:T:20;
r =0.1*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
plot(t,y,'r');

% Precompensator design
N = -35000000;
sys_cl = ss(Ac,Bc*N,Cc,Dc,T);
t = 0:T:20;
r =0.1*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
plot(t,y,'r',t,r,'b');