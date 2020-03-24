%% Exercise no. 7 (PART - b) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 07.11.2018

%% Genetic algorithm
clc
clear all
close all

s = tf('s');
%% System transfer
G_s   = 1/(s*(s+1)*(s+2));

%% Zieglera nichols - Type 2
r0k = 6;
K_cr = r0k; % critical gain
wk  = sqrt(2);
P_cr  = 2*pi/wk; % period on critical gain

Kp = 0.6*K_cr; 
Ti = 0.5*P_cr;  % [ s ]
Td = 0.12*P_cr; % [ s ]

G_r_ZN_2 = Kp*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_2_b = 0.075*r0k*Tk*(((s + 4/Tk)^2)/s)
%% PID tuner
G_R_1 = pidtune(G_s,'PID');

%Kp_pidT = 3;
%Ki_pidT = 0.345;
%Kd_pidT = 3.85;

%G_r_pidT = Kp_pidT + Ki_pidT/s + Kd_pidT*s;

G_r_pidT = tf(G_R_1);
%% PID - Genetic algorithm
Kp_GA_ITAE = 62.7021760827214;
Ki_GA_ITAE = 35.02639745238779;
Kd_GA_ITAE = 99.99999826059673;

GA_PID_ITAE = Kp_GA_ITAE + Ki_GA_ITAE/s + Kd_GA_ITAE*s;

Kp_GA_IAE = 3.840231459190198;
Ki_GA_IAE = 0.15544802795476897;
Kd_GA_IAE = 99.99998860290594;

GA_PID_IAE = Kp_GA_IAE + Ki_GA_IAE/s + Kd_GA_IAE*s;

%options = gaoptimset
%Fitness_fce_itae = @(x)itae_ga(x);
%Fitness_fce_iae  = @(x)iae_ga(x);
%[...] = ga(...); 

%% total system transfer
G_t_pidT      = feedback(G_r_pidT*G_s,1);
G_t_pidZN_2   = feedback(G_r_ZN_2*G_s,1);
GA_PID_C_ITAE = feedback(GA_PID_ITAE*G_s,1);
GA_PID_C_IAE  = feedback(GA_PID_IAE*G_s,1);

t   = 0:0.01:50; % time

step(G_t_pidT, G_t_pidZN_2,GA_PID_C_ITAE,GA_PID_C_IAE,t);
legend('PID tool','Ziegler–Nichols Method 2','GA PID with FF ITAE','GA PID with FF IAE')
grid on