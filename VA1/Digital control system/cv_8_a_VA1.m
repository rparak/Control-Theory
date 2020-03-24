%% Exercise no. 8 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 14.11.2018

%% Discrete systems
clc
clear all
close all

%% Initialization Discrete system
Ts = 0.1; % sampling period
z = tf('z', Ts);

%% System transfer
G_z1 = (z - 1)/(z^2 - 1.85*z + 0.9); % positive shift
G_z2 = (1 - 1*z^(-1))/(1 - 1.85*z^(-1) + 0.9*z^(-2)); % negative shift

G_s  = d2c(G_z1); % discrete to contiuous
G_zN = c2d(G_s, Ts); % continuous to discrete

%% total system transfer
step(G_z_a, G_z_b)