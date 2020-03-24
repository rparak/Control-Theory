%% Exercise no. 4 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory II
% Date: 26.2.2018

%% Developing a state space model from a system diagram (Mechanical Translating
clc
clear all
close all

%% Initialization parameters
m = 0.2;
b = 0.2;
k = 1;

%% State space - equations (Continuous-time system)
A = [0 1; -k/m -b/m];

B = [b/m; (k/m - (b/m)^2)];
 
C = [1 0];

D = [0];

% Resulting transfer function (Continuous-time transfer function)
s_c = ss(A, B, C, D);

%% Stability calculation
% PART A: using the MATLAB function
s_sys = isstable(s_c);

% PART B: calculation
p_sys = eig(A); % finding poles

e_s = 0;

for i = 1:length(p_sys)
    if p_sys(i) > 0
        e_s = 1;
        break
    end
end

if e_s
    disp('Given System is unstable.')
else
    disp('Given System is stable.')
end

%% Matrix Rank
n  = rank(A);

%% Controllable Matrix
% To Find Controllable Matrix Mc, it's rank and check controllability.
Mc = ctrb(A, B);
rank_Mc = rank(Mc);

if (rank_Mc ~= n && rank_Mc ~= 0)
    error('Given System is Uncontrollable.')
else
    disp('Given System is Controllable.')
end

%% Observable Matrix
% To Find Observable Matrix Mo, its rank and check observability.
Mo = obsv(s_c);
rank_Mo = rank(Mo);

if (rank_Mo ~= n && rank_Mo ~= 0)
    error('Given System is Unobservable.')
else
    disp('Given System is Observable.')
end

%% Discrete-time system
% Sample time
Ts = 0.25;

% Transfer to  Discrete-time transfer function
s_d = c2d(s_c, Ts);

%% Plot - result
subplot(2,1,1)
step(s_c)
grid on
subplot(2,1,2)
step(s_d)
grid on