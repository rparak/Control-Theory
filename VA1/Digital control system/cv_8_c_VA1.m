%% Exercise no. 8 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 14.11.2018

%% Discrete systems (Method of optimal module -> PI/PS controller)
clc
clear all
close all

%% Initialization Continuous system
s = tf('s');

%% System transfer (Continous)
G_s = 2/((10*s + 1)*(5*s +1));

k1 = 2;
T1 = 10;
T2 = 5;

%pzmap(G_s) % check stability

%% Computing PID controller
Ti  = T1;
r0  = Ti/(2*k1*T2);
pi_cs = r0*(1 + 1/(Ti*s));

G_continuous = feedback(pi_cs*G_s, 1);

%% System transfer (Discrete)
T = 1;
z  = tf('z', T);

G_d = c2d(G_s,T,'zoh'); % Zero-order hold (default), 'foh' — Triangle approximation

%pzmap(G_d) % check stability

%% Computing PSD controller
Ti = T1 - 0.5*T;
r0 = Ti/(2*k1*T2);

q0 = r0*(1 + 1/Ti);
q1 = -r0;

ps_cd = (q0 + q1*z^(-1))/(1 - z^(-1));

G_digital = feedback(ps_cd*G_d, 1);

%% total system transfer
step(G_continuous, G_digital)