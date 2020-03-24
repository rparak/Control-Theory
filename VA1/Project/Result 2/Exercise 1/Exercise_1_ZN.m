%% Ziegler and Nichols for exercise 1

clc
clear all
close all

%Given values
m1 = 1.35;
m2 = 1.45;
k1 = 0.95;
k2 = 1.05;
b = 0.25;

% Transfer function
s = tf('s');
G_s = (k1*k2+k1*b*s)/(m1*m2*s^4+b*(m1+m2)*s^3+(m2*k2+m2*k1+m1*k2)*s^2+s*k1*b+k1*k2);
K_cr = 1.05;
wk  = 0.86;
P_cr  = 2*pi/wk;

%% PID - controller
Kp = 0.6*K_cr
Ti = 0.5*P_cr  % [ s ]
Td = 0.12*P_cr % [ s ]

G_r_ZN_2 = Kp*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_2_b = 0.075*r0k*Tk*(((s + 4/Tk)^2)/s)
%% total system transfer
G_t_pidZN_SL = feedback(G_r_ZN_2*G_s,1);
t   = 0:0.005:100;

step(G_t_pidZN_SL,t);
xlabel('time'); 
ylabel('H_s');
title('Step response, PID designed by Ziegler & Nichols'); 