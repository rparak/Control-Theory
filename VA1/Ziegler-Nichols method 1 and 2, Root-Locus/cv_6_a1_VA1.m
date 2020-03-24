%% Exercise no. 6 (PART - 1a) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 31.10.2018

%% Ziegler-Nichols Method 1: Mass-spring-damper system
clc
clear all
close all

s = tf('s');
%% System transfer
m = 1;  % [ kg ]
b = 10; % [ N.s/m ]
k = 20; % [ N/m ]

G_s = 1/(m*s^2 + b*s + k);
step(G_s);
grid on
%% Zieglera nichols - Type 1
dt = 0.01;
t  = 0:dt:10;
y  = step(G_s,t);

dy = diff(y)/dt; % derivation
[m,idx] = max(dy);
yi = y(idx);
ti = t(idx);

Kp = y(end); % gain
Tu = ti - yi/m; % time delay
Tn = (Kp-yi)/m+ti-Tu; % time constant

plot(t,y,'b--',[Tu Tu+Tn Tu+Tn],[0 Kp 0])
grid on
%% PID - controller
r0 = 1.2*1/Kp*(Tn/Tu);
Ti = 2*Tu;   % [ s ]
Td = 0.5*Tu; % [ s ]

G_r_ZN_1 = r0*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_1_b = 0.6*Tn*(((s + 1/Tu)^2)/s);
%% total system transfer
G_t_pidZN_1 = feedback(G_r_ZN_1*G_s,1);

t = 0:0.01:10;

step(G_t_pidZN_1,t);
legend('Ziegler–Nichols Method 1')
grid on