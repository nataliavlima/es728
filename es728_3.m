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
sysq2  = ss(A,B,C,D);
C = [1 0 0 0];
sysq1 = ss(A,B,C,D);  
observability_q2 = obsv(sysq2);
rank_obs_q2 = rank(observability_q2)

controlability_q2 = ctrb(sysq2);
rank_con_q2 = rank(controlability_q2)

observability_q1 = obsv(sysq1);
rank_obs_q1 = rank(observability_q1)
controlability_q1 = ctrb(sysq1);
rank_con_q1 = rank(controlability_q2)

%% 
 % Controller
des_poles = [-1, -4, -7, -10];
K=acker(A,B,des_poles)
eig(A-B*K)
C = eye(4);
%Q=20*eye(4);
% R=8;
% K_lqr2=lqr(A,B,Q,R);
% eig(A-B*K_lqr2)
 %K_lqr =  -14.1421  171.8846  -25.2787   54.7535

 %% Second controller

 des_poles = [-3, -3, -3, -3];
K=acker(A,B,des_poles)
eig(A-B*K)

%% LQR
Q= diag([15 15 1 1]);
R=0.6;
K_lqr2=lqr(A,B,Q,R)
eig(A-B*K_lqr2)

%% LQR 2

Q= diag([2 2 1 1]);
R=10;
K_lqr2=lqr(A,B,Q,R)
eig(A-B*K_lqr2)