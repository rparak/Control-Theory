%% Exercise no. 6 (PART - 4a) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 31.10.2018

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

%% disturbance-rejection performance, compute the transfer function from v to y
S_2 = feedback(G_S,G_R_2,1);

%% 1-DOF PID controller with the same bandwidth
G_R_1 = pidtune(G_S,'PID',w_b);
G_1   = feedback(G_S*G_R_1,1);
S_1   = feedback(G_S,G_R_1); % disturbance-rejection

%% improve disturbance rejection
opt         = pidtuneOptions('DesignFocus','disturbance-rejection');
Gdr_DOF2_tf = pidtune(G_S,'PID2',w_b,opt);

Gdr_DOF2_tf = tf(Gdr_DOF2_tf);

Gdr_FF      = Gdr_DOF2_tf(1);
Gdr_R_2     = Gdr_DOF2_tf(2);

Gdr_2 = Gdr_FF*feedback(G_S,Gdr_R_2,1);

Sdr_2 = feedback(G_S,Gdr_R_2,1); % disturbance-rejection
%% total system transfer
subplot(2,1,1)
stepplot(G_1,G_2,Gdr_2)
title('Reference Tracking')
legend('1-DOF','2-DOF','2-DOF rejection focus')
grid on
subplot(2,1,2)
stepplot(S_1,S_2,Sdr_2)
title('Disturbance Rejection')
grid on
legend('1-DOF','2-DOF','2-DOF rejection focus')