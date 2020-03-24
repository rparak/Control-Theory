%% Ziegler-Nichols applied to exercise 2
clc
clear all
close all

s = tf('s');
%% System transfer
% Given values
C1 = 1*10^(-6);
C2 = 1.15*10^(-6);
R1 = 5*10^5;
R2 = 0.5*10^4; 

% For the transfer function
p0 = 1; 
r0 = 1; 
r1 = R1*C1+R1*C2+R2*C2; 
r2 = R1*R2*C1*C2; 
H_s = p0/(r0 + r1*s + r2*s^2);
G_s = minreal(H_s);

subplot(2, 1, 1); 
step(G_s);
xlabel('time'); 
ylabel('H_s'); 
title('Step response H_s'); 
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

%% PID - controller
r0 = 1.2*1/Kp*(Tn/Tu);
Ti = 2*Tu;   % [ s ]
Td = 0.5*Tu; % [ s ]

G_r_ZN_1 = r0*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_1_b = 0.6*Tn*(((s + 1/Tu)^2)/s);
%% total system transfer
G_t_pidZN_1 = feedback(G_r_ZN_1*G_s,1);

subplot(2, 1, 2); 
step(G_t_pidZN_1);
xlabel('time'); 
ylabel('H_s'); 
title('Step response H_s with Ziegler & Nichols designed PID'); 
grid on

figure(2) 
step(G_t_pidZN_1);
xlabel('time'); 
ylabel('H_s'); 
title('Step response H_s with Ziegler & Nichols designed PID'); 
grid on