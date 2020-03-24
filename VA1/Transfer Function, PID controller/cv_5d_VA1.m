%% Exercise no. 5 (PART - 4) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 24.10.2018

%% P, PI, PD, PID control: Mass-spring-damper system
clc
clear all
close all

%% Intitialization parameters
m  = 1;  % mass             [kg]
k  = 20; % spring constant  [N/m]
b  = 10; % damping constant [Ns/m] Overdamped
F  = 1;  % input force      [N]

% Options set for step
opt = stepDataOptions('InputOffset',0,'StepAmplitude',F);
%% Resulting transfer function (Continuous-time transfer function) - without a controller
transfer_fce_model = tf([1],[m b k]);

transfer_fce_model_Wc = feedback(transfer_fce_model, 1);
t1 = 0:0.01:40;
%% Proportional Control
Kp = 300;

C  = pid(Kp);

transfer_fce_model_Pc = feedback(C*transfer_fce_model, 1);

t2 = 0:0.01:2;
%% Proportional-Derivative Control
Kp = 300;
Kd = 10;

C = pid(Kp,0,Kd);

transfer_fce_model_PDc = feedback(C*transfer_fce_model, 1);

t3 = 0:0.01:2;
%% Proportional-Integral Control
Kp = 30;
Ki = 70;

C = pid(Kp,Ki);

transfer_fce_model_PIc = feedback(C*transfer_fce_model, 1);

t3 = 0:0.01:2;
%% Proportional-Integral-Derivative Control
Kp = 350;
Ki = 300;
Kd = 50;

C = pid(Kp,Ki,Kd);

transfer_fce_model_PIDc = feedback(C*transfer_fce_model, 1);

t4 = 0:0.01:2;
%% Plot - result
subplot(5,1,1)
step(transfer_fce_model_Wc, opt, t1)
title('Without a controller')

subplot(5,1,2)
step(transfer_fce_model_Pc, opt, t2)
title('Proportional Control')

subplot(5,1,3)
step(transfer_fce_model_PDc, opt, t3)
title('Proportional-Derivative Control')

subplot(5,1,4)
step(transfer_fce_model_PIc, opt, t3)
title('Proportional-Integral Control')

subplot(5,1,5)
step(transfer_fce_model_PIDc, opt, t3)
title('Proportional-Integral-Derivative Control')