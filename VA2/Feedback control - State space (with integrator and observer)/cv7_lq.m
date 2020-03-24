%% Exercise no. 7 (PART - 4) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
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

%% State space - equations (Continuous-time system) - Adding integration part 
Ai = [A zeros(n,1); -C 0]; 
Bi = [B; -D]; 
Ci = [C 0];
Br = [0; 0; 1];

% Resulting transfer function (Continuous-time transfer function)
sys = ss(Ai,Bi,Ci,D);

%% Controllable Matrix
% To Find Controllable Matrix Mc, it's rank and check controllability.
Mc = ctrb(sys); 

if rank(ctrb(A,B)) == order(sys)
    fprintf('Given System is Controllable. \n');
end
    
%% Design LQ controller
Q = eye(3,3); % matrix Q must be square with the same order as the system
Q(1,1) = 1e3; % we want to minimize the first state (position)

R = eye(m); % matrix R must be square with the same order as the number of inputs
R = R*10;
K = lqrd(Ai, Bi, Q, R, T); % design of a discrete controller (gain vector) for a continuous-time system

Ki = K(3);   % integration part
Kr = K(1:n); % controller part

% Resulting transfer function (Continuous-time transfer function)
r_tf = ss(Ai - Bi*K, Br, Ci, D)
%% Plot - result
step(r_tf)