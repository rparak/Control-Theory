%% Exercise no. 5 (PART - 1a) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 24.10.2018

%% Mass-spring-damper system
clc
clear all
close all

%% Intitialization parameters
m   = 100;  % mass             [kg]
k   = 100;  % spring constant  [N/m]
k_1 = 1/k;  % spring constant  [N/m]
b1  = 1000; % damping constant [Ns/m] Overdamped
b2  = 200;  % damping constant [Ns/m] Critically damped
b3  = 100;  % damping constant [Ns/m] Underdamped
F   = 1;    % force            [N]

% Options set for step
opt = stepDataOptions('InputOffset',0,'StepAmplitude',F);

t = 0:0.1:1500; % Sample time
%% Resulting transfer function (Continuous-time transfer function)
transfer_fce_model_1N1 = tf([1],[m b1 k]);
transfer_fce_model_2N1 = tf([1],[m b2 k]);
transfer_fce_model_3N1 = tf([1],[m b3 k]);

transfer_fce_model_1N2 = tf([1],[m b1/k k/k]);
transfer_fce_model_2N2 = tf([1],[m b2/k k/k]);
transfer_fce_model_3N2 = tf([1],[m b3/k k/k]);
%% Plot - result
subplot(2,1,1)
step(transfer_fce_model_1N1,transfer_fce_model_2N1,transfer_fce_model_3N1, opt);
legend('Epsilon > 1', 'Epsilon = 1', '0 < Epsilon < 1')

subplot(2,1,2)
step(transfer_fce_model_1N2,transfer_fce_model_2N2,transfer_fce_model_3N2, opt);
legend('Epsilon > 1', 'Epsilon = 1', '0 < Epsilon < 1')