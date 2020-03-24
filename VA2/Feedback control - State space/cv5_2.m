%% Exercise no. 5 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
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
Aa  = [0 1 0 0
       0 0 1 0
       0 0 -b/J K/J
       0 0 -K/L -R/L];
Ba  = [-1 ; 0 ; 0 ; 0 ];
Bau = [0 ; 0 ; 0; 1/L];
Ca  = [0 1 0 0];
Da  = [0];

%% Controller design
p1 = -100+100i;
p2 = -100-100i;
p3 = -150;
p4 = -200; % p4 > p1, p2, p3

Ra   = place(Aa,Bau,[p1,p2,p3,p4]); % or acker

% Resulting transfer function (Continuous-time transfer function)
r_tf = ss(Aa-Bau*Ra, Ba, Ca, Da);
%% Plot - result
t = 0:0.001:0.1;
step(r_tf, t)
grid on