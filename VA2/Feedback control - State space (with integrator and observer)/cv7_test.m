clc
clear all
close all

%% Initialization parameters
m = 0.2;   % mass [kg] 
b = 0.2;   % spring constant [N/m]
k = 1;     % damping constant [Ns/m]

T = 0.01;  % sample time

%% State space - equations (Continuous-time system)
A = [0 1; -k/m -b/m]; 
B = [0; 1/m]; 
C = [1 0]; 
D = 0; 

n = size(B, 1); % control system
m = size(B, 2); % input value

% Resulting transfer function (Continuous-time transfer function)
sys = ss(A,B,C,D);