%% Ziegler-Nichols applied to exercise 3_position
clc
clear all
close all

s = tf('s');
%% System transfer
% Given values
J = 3.2284*10^(-6); 
b = 3.5077*10^(-6); 
K = 0.0274; 
R = 4 ; 
L = 2.75*10^(-6); 

% For the transfer function
p0 = K ;
r0 = 0; 
r1 = R*b+K^2; 
r2 = b*L+R*J;
r3 = J*L; 

%% Transfer function 
 
G_s = p0/(r0 + r1*s + r2*s^2 + r3*s^3); 
G_s = minreal(G_s)

t = 0:0.001:0.2;
subplot(2, 1, 1); 
step(G_s,t); 
xlabel('time'); 
ylabel('G_s'); 
title('Step response G_s'); 
grid on
%% Zieglera nichols - Type 1
dt = 0.001;
t = 0:dt:0.2;
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