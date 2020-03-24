%% Exercise no. 9 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 21.11.2018

%% Circuit with auxiliary controlled variable
clc
clear all
close all

%% Initialization Continuous system
s = tf('s');

%% Initialization parameters
k     = 0.00; % max. overshoot (in percent) -> dependence of coefficient on the beta
beta  = 2.718;

%% 1. Simple control circuit
% system transfer fucntion
G_1   = (4/(2*s + 1)*(0.5*s+1))*exp(-4*s);

k   = 4;
T1  = 2;
T2  = 0.5;
Tdd = 4; % Time delay

%% Calculation PID controller -> G_RH
Ti = T1 + T2;
Td = (T1*T2)/(T1 + T2);

a  = 1/(beta*Tdd);
r0 = (a*Ti)/k;

pid_C1        = r0*(1 + 1/(Ti*s) + Td*s);
G_RH_simple   = pid(pid_C1);
vz
%% 2. Control circuit with auxiliary controlled variable
% system transfer fucntions
G_s1 = 4/(0.5*s + 1);
G_s2 = (1/(2*s + 1))*exp(-4*s);

%% Calculation PID controller -> G_RH, PI controller -> G_RP
% a) PI controller -> G_RP
T1_s1 = 0.5;
Ti_s1 = T1_s1;
k_s1  = 4;
Tw_s1 = calc_tw(G_s1);

r0_s1 = 2*Ti_s1/(2*k_s1*Tw_s1);
pi_C1 = r0_s1*(1 + 1/(Ti_s1*s));

G_RP = pid(pi_C1);

G_s1N  = feedback(pi_C1*G_s1,1);
G_s1N2 = minreal(G_s1N);

% b) PID controller -> G_RH
G_sNN  = (1/(0.5*s + 2)*(2*s + 1))*exp(-4*s);

k_NN   = 1;
T1_nn  = 2;
T2_nn  = 0.5;
Tdd_nn = 4;

Ti_nn  = T1_nn + T2_nn;
Td_nn  = (T1_nn*T2_nn)/(T1_nn + T2_nn);

a_nn   = 1/(beta*Tdd_nn);
r0_nn  = (a_nn*Ti_nn)/k_NN;

pid_C1_nn = r0_nn*(1 + 1/(Ti_nn*s) + Td_nn*s);
G_RH      = pid(pid_C1_nn);