clc;clear;close all force;

%% Define model parameters
mc = 1.5; % mass of the cart
mp = 0.5; % mass of the pendulum
g = 9.82; % gravity
L = 1;    % length of the pendulum
d1 = 1e-2;  % damping of the cart displacement
d2 = 1e-2; % damping of the joint

%% Modelo de espaco de estado
A = [0,   0,   1,    0;
     0,   0,   0,    1;
     0,   g*mp/mc,   -d1/mc, -d2/(L*mc);
     0,   g*(mc+mp)/(L*mc),  -d1/(L*mc), -d2*(mc+mp)/(L^2*mc*mp)];
 
 B = [ 0; 0; 1/mc;  1/(L*mc)];
 
 C = [0 1 0 0];
     
 D = 0;
 
 % Build system
 sysq2  = ss(A,B,C,D)
 %rlocus(sys)% tem um polo positivo, logo o sistema nao eh estavel
 
%% Polos, autovalores e zeros

pole_q1 = pole(sysq2);
zero_q1 = zero(sysq2);
eigenvalue_q1 = eig(sysq2);
%portanto o polo em 0 tornaria o sistema marginalmente estavel
%o polo em 3,6 torna-o nao estavel. Os zeros estão praticamente em 0
%Os autovalores equivalem aos polos.

pole_ = [-3.6327; 3.6043; 0; 0];
zero_ = [0; 0];
gain_ = 1;
[b,a] = zp2tf(zero_, pole_, gain_);
tf_ = tf(b,a)
%nao entendi pq houve cancelamento de polos e zeros. na pratica
%transformei eles em zero, eh obvio que sumiriam.

%% FT e saida q1

[b_,a_] = ss2tf(A,B,C,D);
tf_q2 = tf(b_,a_)

C_q1 = [1 0 0 0];
sysq1 = ss(A, B , C_q1, D);
[b,a] = ss2tf(A,B,C_q1,D);
tf_q1 = tf(b,a)
%tf_q1 e tf_q2 sao diferentes

%% Polos e zeros entre FTs para q1 e q2

pole_q1 = pole(sysq1)
zero_q1 = zero(sysq1)

pole_q2 = pole(sysq2)
zero_q2 = zero(sysq2)

%ha diferenca. os polos, como esperado, sao iguais. porem os zeros diferem
%a ft de ambas difere no numerador( de onde se obtem os polos). Essa
%diferenca vem da mudanca da matriz de saida C

% Controller
%  des_poles = 2*[-1, -1, -1, -1];
%  K=acker(A,B,des_poles)
%  eig(A-B*K)
% Q=20*eye(4);
% R=8;
% K_lqr2=lqr(A,B,Q,R);
% eig(A-B*K_lqr2)
 %K_lqr =  -14.1421  171.8846  -25.2787   54.7535
 
 