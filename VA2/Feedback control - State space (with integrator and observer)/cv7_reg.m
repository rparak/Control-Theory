%% Exercise no. 7 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 19.3.2018

%% Developing a state space model from a system diagram (Mechanical Translating)
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

%% Controllable Matrix
% To Find Controllable Matrix Mc, it's rank and check controllability.
Mc = ctrb(sys); 

if rank(ctrb(A,B)) == order(sys)
    fprintf('Given System is Controllable. \n');
end

%% Discrete-time system
sys_d = c2d(sys,T); % transfer to  Discrete-time transfer function

pole(sys_d); % poles detection {open loop}

% desigin poles {closed loop} - Rules {stability, speed, no overshoot}
op = [-0.8 -0.9]; 

K = place(sys_d.A, sys_d.B, op); % or acker {design controller}

% Resulting transfer function (Continuous-time transfer function)
r_tf = ss(A-B*K, B, C, D);
%% Plot - result
step(r_tf)