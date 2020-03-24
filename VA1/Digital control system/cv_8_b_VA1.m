%% Exercise no. 8 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 14.11.2018

%% Discrete systems (Dynamics inversion design -> PID/PSD controller)
clc
clear all
close all

%% Initialization Continuous system
s = tf('s');

%% Initialization parameters
k = 0.05; % max. overshoot (in percent) -> dependence of coefficient on the alpha, beta
beta  = 1.944;
alpha = 0.984;

%% System transfer (Continous)
G_s = 2/((5*s + 1)*(2*s+1))*exp(-12*s);

Tdd = 12; % time delay
T1 = 5;
T2 = 2;
k1 = 2;
%pzmap(G_s) % check stability

%% Computing PID controller
a  = 1/(beta*Tdd);
Ti = T1 + T2;
Td = (T1*T2)/(T1 + T2);
r0 = (a*Ti)/(k1);

pid_cs = r0*(1 + 1/(Ti*s) + Td*s);

G_continuous = feedback(pid_cs*G_s,1);

%% System transfer (Discrete)
T = 0.32*Tdd;
T = round(T - 2); % % sampling period

z = tf('z', T);

G_d = c2d(G_s,T,'zoh'); % Zero-order hold (default), 'foh' — Triangle approximation

% pzmap(G_d) % check stability
%% Computing PSD controller
a_d = 1/(alpha*T + beta*Tdd);
Ti = T1 + T2 - T;
Td = (T1*T2)/(T1 + T2) - T/4;
r0 = (a_d*Ti)/(k1);

q0     = r0*(1+Td/T + T/Ti);
q1     = -r0*(1+2*Td/T);
q2     = r0*Td/T;

psd_cd = (q0 + q1*z^(-1) + q2*z^(-2))/(1-z^(-1));

G_digital = feedback(psd_cd*G_d, 1);

%% total system transfer
step(G_continuous,G_digital)

