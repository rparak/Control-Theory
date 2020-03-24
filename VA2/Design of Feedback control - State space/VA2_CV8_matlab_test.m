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