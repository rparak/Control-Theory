%% Exercise no. 9 (PART - 3) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 21.11.2018

%% Circuit with auxiliary measurement of disturbance variable
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
G_3   = (4/(8*s + 1)*(0.2*s+1))*exp(-6*s);

k   = 4;
T1  = 8;
T2  = 0.2;
Tdd = 6; % Time delay

%% Calculation PID controller -> G_RH
Ti = T1 + T2;
Td = (T1*T2)/(T1 + T2);

a  = 1/(beta*Tdd);
r0 = (a*Ti)/k;

pid_C1 = r0*(1 + 1/(Ti*s) + Td*s);
G_RH = pid(pid_C1);

%% 2. Control circuit with auxiliary controlled variable
%system transfer fucntions
G_2_s1 = 2/(0.2*s + 1);
G_2_s2 = (2/(8*s + 1))*exp(-6*s);

%% Calculation controller -> G_RP
G_SNN = 1/G_2_s1;

G_RP = pid(G_SNN);