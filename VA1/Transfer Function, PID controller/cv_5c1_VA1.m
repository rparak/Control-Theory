%% Exercise no. 5 (PART - 3a) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
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
Gp_21 = 1/((s + 1)*(3*s + 1));
Gp_22 = 1/((s + 1)^2);
Gp_24 = 1/(s^2 + s + 1);

%% Plot - result {P}
subplot(1,3,1)
step(Gp_1,Gp_22,Gp_24,t);
grid on
title('P - controller')
%% I - controller
Gi_1   = 1/s;
Gi_11  = 1/(s*(s + 1));
Gi_n12 = (1 - 10*s)/(s*(2*s+1)*(3*s+1)); % positive zero {first on the other side}
%% Plot - result {I}
subplot(1,3,2)
step(Gi_1,Gi_11,Gi_n12,t);
grid on
title('I - controller')
%% D - controller 
Gd_11 = (10*s)/(3*s + 1);
Gd_12 = (10*s)/((2*s + 1)*(3*s + 1));
Gd_23 = (10*s^2)/((2*s + 1)*(3*s + 1)*(4*s + 1));
%% Plot - result {D}
subplot(1,3,3)
step(Gd_11,Gd_12,Gd_23,t);
grid on
title('D - controller')