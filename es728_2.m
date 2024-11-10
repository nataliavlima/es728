clc;clear;close all force;

%a primeira parte deve ser feita no simulink. Como to no matlab online n
%mexi nisso ainda
%% Define model parameters
mc = 1.5; % mass of the cart
mp = 0.5; % mass of the pendulum
g = 9.82; % gravity
L = 1;    % length of the pendulum
d1 = 1e-2;  % damping of the cart displacement
d2 = 1e-2; % damping of the joint

%% Lugar das raizes.
A = [0,   0,   1,    0;
     0,   0,   0,    1;
     0,   g*mp/mc,   -d1/mc, -d2/(L*mc);
     0,   g*(mc+mp)/(L*mc),  -d1/(L*mc), -d2*(mc+mp)/(L^2*mc*mp)];
 
 B = [ 0; 0; 1/mc;  1/(L*mc)];
 
 C = [0 1 0 0];
     
 D = 0;
 
 % Build system
 sysq2  = ss(A,B,C,D)
 rlocus(sysq2)% tem um polo positivo, logo o sistema nao eh estavel
 %nao da pra estabilizar com um controlador P, pois ele não altera a
 %posição dos polos no lugar das raízes.

 %% sisotool
%sisotool(sysq2)

b = [1793.9 1793.9*12 1793.9*100];
a = [1 40 0];
gain = 1;
C_s = tf(b,a);
plant = feedback(C_s*sysq2,1);
step(plant);