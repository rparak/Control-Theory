%% Exercise no. 3 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 19.2. 2018

%% Developing a state space model from a system diagram (Mechanical Translating)
clc
clear
close all;

% Initialization parameters
m = 1;
b = 10;
k = 1;
F = 1000;

% State space - equations (Continuous-time system)
A = [0 1; -k/m -b/m];

B = [0; 1/m];
 
C = [1 0];

D = [0];

% Resulting transfer function (Continuous-time transfer function)
s_c = ss(A, B, C, D);

% Sample time
Ts = 1.5;

% Transfer to  Discrete-time transfer function
s_d = c2d(s_c, Ts);

A_d = s_d.A;
B_d = s_d.B;
C_d = s_d.C;
D_d = s_d.D;

% finding poles
psys = eig(A);

%% Plot - result
subplot(2,1,1)
step(s_c)
grid on
title('Continuous-time transfer function')
subplot(2,1,2)
step(s_d)
grid on
title('Dicrete-time transfer function')
