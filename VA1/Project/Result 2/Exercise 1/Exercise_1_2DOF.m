%% 2-DOF controller
clc
clear all
close all

%% System transfer
G_S = tf([0.1213 0.5096],[1 0.3576 2.206 0.1213 0.5096]);

%% To find Ki, Kd, Kp, b and c 
pidtool(G_S); 

%% decompose the controller into the components Gff and Gr, and use them to compute the closed-loop response from w to y

s = tf('s'); 
Kp = 0.079561;
Ki = 0.0056939; 
Kd = 0.27793; 
b = 1; 
c = 1; 
G_FF      = b+(1/(Ki*s))+c*Kd*s
G_R_2     = -Kp*(1+(1/(Ki*s))+Kd*s)

G_2 = G_FF*feedback(G_S,G_R_2,1);

%% total system transfer
step(G_2)
title('Reference Tracking')
grid on