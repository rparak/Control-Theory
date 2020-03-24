%% Exercise no. 5 (PART - 1b) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 24.10.2018

%% P, PI, PD, PID control: Mass-spring-damper system
clc
clear all
close all

%% Intitialization parameters
m  = 1;   % mass             [kg]
k  = 1;   % spring constant  [N/m]
b  = 0.2; % damping constant [Ns/m] Overdamped
F  = 1;   % input force      [N]

% Options set for step
opt = stepDataOptions('InputOffset',0,'StepAmplitude',F);
%% Resulting transfer function (Continuous-time transfer function) - without a controller
transfer_fce_model = tf([1],[m b k]);
%% Plot - result
step(transfer_fce_model, opt)