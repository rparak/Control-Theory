%% Exercise no. 6 (PART - 3) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 31.10.2018

%% Root-Locus
clc
clear all
close all

s = tf('s');
%% System transfer
sys = (s + 7)/(s*(s + 5)*(s + 15)*(s + 20));
rlocus(sys)
axis([-22 3 -15 15])

%% Zeta, omega computing
zeta = 0.7; % from graph -> max 5% overshoot
wn   = 1.8; % tr = 1.8/wn < 1s -> rise time 1s
sgrid(zeta,wn)

%[k,poles] = rlocfind(sys); % manual find from graph (matlab function)

%% Find s from root-locus graph
s1 = -2.84+0.958i;
sys1 = (s1 + 7)/(s1*(s1 + 5)*(s1 + 15)*(s1 + 20));
K1 = 1/(abs(sys1));

s2 = -2.88+1.39i;
sys2 = (s2 + 7)/(s2*(s2 + 5)*(s2 + 15)*(s2 + 20));
K2 = 1/(abs(sys2));

%K1 < K < K2
K = 347;

sys_cl = feedback(K*sys,1);

%% total system transfer
step(sys_cl)

%% toolbox
% controlSystemDesigner(sys_cl)
% pidTuner