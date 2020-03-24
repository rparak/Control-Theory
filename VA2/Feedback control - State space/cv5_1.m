%% Exercise no. 5 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 5.3.2018

%% Developing a state space model from a system diagram (Mechanical Translating)
clc
clear all
close all

%% Initialization parameters
J  = 0.01; % moment of inertia of the rotor [ kg.m^2 ] 
b  = 0.1;  % motor viscous friction constant [ N.m.s ]
Ke = 0.01; % electromotive force constant [ V/rad/sec]
Kt = 0.01; % motor torque constant [ N.m/Amp ] 
R  = 1;    % electric resistance [ Ohm ]
L  = 0.5;  % electric inductance [ H ]

K = Ke;
%% State space - equations (Continuous-time system)
A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];
B = [0 ; 0 ; 1/L];
C = [1 0 0];
D = [0];

% Resulting transfer function (Continuous-time transfer function)
motor_ss = ss(A,B,C,D);
% rlocus(motor_ss)
%% Stability calculation
% PART B: calculation
p_sys = eig(A); % finding poles

%% Matrix Rank
n  = rank(A);

%% Controllable Matrix
% To Find Controllable Matrix Mc, it's rank and check controllability.
Mc        = ctrb(motor_ss);
rank_Mc   = rank(Mc);

X = motor_ss.A; Y = motor_ss.B;

%% Controller design 1
% dominant eigenvalue (c1) with the multiplication factor (k) 
c1 = -150; k = 1.1; 
c = [1 k k^2]*c1;

R  = place(A,B,[c(1), c(2), c(3)]);
%% Controller design 2
p1 = -100+100i;
p2 = -100-100i;
p3 = -100;

R  = place(A,B,[p1, p2, p3]);

%% Resulting transfer function (Continuous-time transfer function)
r_tf = ss(A-B*R, B, C, D);
%% Plot - result
t = 0:0.001:0.1;
step(r_tf, t)
grid on