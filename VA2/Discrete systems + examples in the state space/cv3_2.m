%% Exercise no. 3 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 19.2. 2018

%% RLC circuit (electrical circuit) - resistor (R), an inductor (L), and a capacitor (C)
clc
clear
close all;

% Initialization parameters
R = 1;
L = 0.2;
C = 100E-06;

% State space - equations (Continuous-time system)
A = [0, -1/L; 1/C, -1/(R*C)];

B = [1/L, -1/L; 0, -1/(R*C)];

C = [1, 0; 1 -1/R];

D = [0, 0; 0, -1/R];

% Resulting transfer function (Continuous-time transfer function)
s_c = ss(A,B,C,D);

% Sample time
Ts = 29/15;

% Transfer to  Discrete-time transfer function
s_d = c2d(s_c, Ts);

A_d = s_d.A;
B_d = s_d.B;
C_d = s_d.C;
D_d = s_d.D;

% finding poles
psys = eig(A);

%% plot - result
subplot(2,1,1)
step(s_c)
grid on
title('Continuous-time transfer function')
subplot(2,1,2)
step(s_d)
grid on
title('Dicrete-time transfer function')