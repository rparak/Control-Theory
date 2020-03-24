%% Exercise no. 8 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 25.3.2019

%% Developing a state space model from a system diagram (Mechanical Translating)
clc
close all
clear all

%% State space - equations (Continuous-time system) > Linearized
A =[ 0       1   0       0;
    -0.3421  0   6.592   0.031;
     0       0   0       1;
     18.898  0  -0.344  -1.713];
B = [0;    -0.0633;   0;   3.4960];
C =  eye(4);
D =  zeros(4,1); 

n = size(B, 1); % control system
m = size(B, 2); % input value

system = ss(A,B,C,D);
%% Observable Matrix / Controllable Matrix
% To Find Controllable Matrix Mc, it's rank and check controllability.
if rank(ctrb(A,B)) == order(system)
    fprintf('Given System is Controllable. \n');
end

% To Find Observable Matrix Mo, its rank and check observability.
if rank(obsv(A,C)) == order(system)
    fprintf('Given System is Observable. \n');
end

%% Conditions
weight=5.41; % [N]  torque/r

% torque |M|<7 N.m
maxR=7/weight; % maximum controlled position (positive/negative direction)

%% State space - equations (Continuous-time system) - Adding integration part 
cp = [-7.5 -3.5 -4 -5.5 -6];
  
Ci = [1 0 0 0];

Ai = [A zeros(4,1); -Ci 0];
Bi = [B; 0];
K = place(Ai, Bi, cp);
Ki = -K(5); % integration part
Kr = K(1:n); % controller part

Brn = [0; 0; 0; 0; 1];
Cin = [1 0 0 0 0];
Dn  = [0];
%% regulation by the observer
observer_poles = 1.8 * cp(1:4); % the poles of the observer must be faster than the poly system

H = place(A', C', observer_poles)'; % matrix of the observer


%% Plot - result
% input -> output
r_tf = ss(Ai - Bi*K, Brn, Cin, Dn)

% input -> 4 output
P = Ai - Bi*K;
L = Brn;
ER = eye(5);
EL = zeros(5,1);

r_tf4 = ss(P, L, ER, EL)
step(r_tf4)