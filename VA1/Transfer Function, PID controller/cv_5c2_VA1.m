%% Exercise no. 5 (PART - 3b) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 24.10.2018

%% P,I,D controller
clc
clear all
close all
%% Initialization transfer fce. and sample time
s = tf('s');
t = 0:0.1:40;
%% P - controller
Gp_1  = 1/(s + 1);
%% Plot - result {P}
subplot(1,3,1)
step(Gp_1 ,t);
grid on
title('P - controller')
%% I - controller
Gi_1   = 1/s;
%% Plot - result {I}
subplot(1,3,2)
step(Gi_1, t);
grid on
title('I - controller')
%% D - controller 
Gd_1 = (10*s)/(3*s + 1);
%% Plot - result {D}
subplot(1,3,3)
step(Gd_1,t);
grid on
title('D - controller')
