%% Exercise no. 9 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 21.11.2018

%% Circuit with auxiliary manipulated variable
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
G_2   = (4/(4*s + 1)*(0.5*s+1))*exp(-2*s);

k   = 4;
T1  = 4;
T2  = 0.5;
Tdd = 2; % Time delay

%% Calculation PID controller -> G_RH
Ti = T1 + T2;
Td = (T1*T2)/(T1 + T2);A

a = 1/(beta*Tdd);
r0 = (a*Ti)/k;

pid_C1    = r0*(1 + 1/(Ti*s) + Td*s);
G_RH = pid(pid_C1);

%% 2. Control circuit with auxiliary controlled variable
% system transfer fucntions
G_2_s1 = (2/(4*s + 1))*exp(-2*s);
G_2_s2 = 2/(0.5*s + 1);

%% Calculation PI controller -> G_RP
T1_s1 = 0.5;
Ti_s1 = T1_s1;
k_s1  = 2;
Tw_s1 = calc_tw(G_2_s2);

r0_s1 = 2*Ti_s1/(2*k_s1*Tw_s1);

pi_C1 = r0_s1*(1 + 1/(Ti_s1*s));

G_RP = pid(pi_C1);