%% Exercise no. 6 (PART - 1b) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 31.10.2018

%% Ziegler-Nichols Method 2
clc
clear all
close all

s = tf('s');
%% System transfer
G_s   = 1/(s*(s+1)*(s+2)); % 1/(s^3 + 3*s^2 + 2*s)

%% Zieglera nichols - Type 2
% r0/(s^3 + 3*s^2 + 2*s) -> s^3 + 3*s^2 + 2*s + r0k = 0
% H2  = [3 r0k;1 2]; % determinant H2 (Hurwitz stability criterion)
r0k = 6;
K_cr = r0k; % critical gain
% (jw)^3 + 3*(jw)^2 + 2*(jw) + 6 = 0;
% -3w^2 + 6   = 0; -> w = sqrt(2)
% -w^3  + 2*w = 0; -> w = sqrt(2)
wk  = sqrt(2);
P_cr  = 2*pi/wk; % period on critical gain
%% PID - controller
Kp = 0.6*K_cr; 
Ti = 0.5*P_cr;  % [ s ]
Td = 0.12*P_cr; % [ s ]

G_r_ZN_2 = Kp*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_2_b = 0.075*r0k*Tk*(((s + 4/Tk)^2)/s)
%% total system transfer
G_t_pidZN_SL = feedback(G_r_ZN_2*G_s,1);
t   = 0:0.01:50;

step(G_t_pidZN_SL,t);