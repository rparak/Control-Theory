%% Exercise no. 7 (PART - 1a) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 12.11.2018

%% 2-DOF controller
clc
clear all
close all

%% System transfer
G_S = tf(1,[1 0.5 0.1]);
%% 2-DOF PID
% pidtool -> check bandwidth
w_b = 1.5; % suppose that your target bandwidth for the system is 1.5 rad/s
[G_DOF2, info_DOF2] = pidtune(G_S,'PID2',w_b);

%% decompose the controller into the components Gff and Gr, and use them to compute the closed-loop response from w to y
G_DOF2_tf = tf(G_DOF2);

G_FF      = G_DOF2_tf(1);
G_R_2     = G_DOF2_tf(2);

G_2 = G_FF*feedback(G_S,G_R_2,1);
%% 1-DOF PID controller with the same bandwidth
G_R_1 = pidtune(G_S,'PID',w_b);
G_1   = feedback(G_S*G_R_1,1);

%% total system transfer
step(G_1,G_2)
title('Reference Tracking')
legend('1-DOF','2-DOF')
grid on