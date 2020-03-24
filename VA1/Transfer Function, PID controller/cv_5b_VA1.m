%% Exercise no. 5 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 24.10.2018

%% Mass-spring-damper system mounted on a massless cart
clc
clear all
close all

%% Intitialization parameters
m  = 1;   % mass             [kg]
k  = 1;   % spring constant  [N/m]
b  = 0.2; % damping constant [Ns/m] Overdamped
F  = 1;   % force            [N]

% Options set for step
opt = stepDataOptions('InputOffset',0,'StepAmplitude',F);

t = 0:0.1:80; % Sample time
%% Resulting transfer function (Continuous-time transfer function)
transfer_fce_model = tf([b k],[m b k]);
%% Plot - result
step(transfer_fce_model, opt, t)