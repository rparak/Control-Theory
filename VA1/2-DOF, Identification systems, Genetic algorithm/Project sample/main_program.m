%% Exercise no. 7 (project guide part) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 12.11.2018

%% Example
clc
clear all
close all

%% Initialization transfer fce. and sample time
s   = tf('s');
t   = 0:0.1:20;
w_b = 4.5;

G_s = (6*s + 3)/(0.5*s^3 + 4*s^2 + 0.5*s + 2);

%% Stability
pzmap(G_s)

p_G_s = pole(G_s);
e     = 0;

for i = 1:length(p_G_s)
    if p_G_s(i) > 0
        disp('System is not stable!');
        e = 1;
    end
end

if e ~= 1
    disp('System is stable!');
end
%% Resulting transfer function (Continuous-time transfer function) - without a controller
G_WC = feedback(G_s, 1);

%% Resulting transfer function (Continuous-time transfer function) - PD tuner {1-DOF}
G_DOF1 = pidtune(G_s,'PD',w_b);

G_1 = feedback(G_s*G_DOF1,1);

%% Resulting transfer function (Continuous-time transfer function) - PID tuner {2-DOF}
[G_DOF2, info_DOF2] = pidtune(G_s,'PID2',w_b);

G_DOF2_tf = tf(G_DOF2);

G_FF      = G_DOF2_tf(1);
G_R_2     = G_DOF2_tf(2);

G_2 = G_FF*feedback(G_s,G_R_2,1);

%% Zieglera nichols - Type 1
dt = 0.01;
t  = 0:dt:10;
y  = step(G_s,t);

dy = diff(y)/dt; % derivation
[m,idx] = max(dy);
yi = y(idx);
ti = t(idx);

Kp = y(end); % gain
Tu = ti - yi/m; % time delay
Tn = (Kp-yi)/m+ti-Tu; % time constant

r0 = 1.2*1/Kp*(Tn/Tu);
Ti = 2*Tu;   % [ s ]
Td = 0.5*Tu; % [ s ]

G_r_ZN_1 = r0*(1 + 1/(Ti*s) + Td*s); % G_r_pidZN_1_b = 0.6*Tn*(((s + 1/Tu)^2)/s);

G_t_pidZN_1 = feedback(G_r_ZN_1*G_s,1);

%% PID - Genetic algorithm
Kp_GA_ITAE = 50.51034576396997;
Ki_GA_ITAE = 49.99996023226804;
Kd_GA_ITAE = 49.999996603002394;

GA_PID_ITAE = Kp_GA_ITAE + Ki_GA_ITAE/s + Kd_GA_ITAE*s;

Kp_GA_IAE = 99.99987858834808;
Ki_GA_IAE = 99.99784555448838;
Kd_GA_IAE = 99.99994737203696;

GA_PID_IAE = Kp_GA_IAE + Ki_GA_IAE/s + Kd_GA_IAE*s;

GA_PID_C_ITAE = feedback(GA_PID_ITAE*G_s,1);
GA_PID_C_IAE  = feedback(GA_PID_IAE*G_s,1);
%% Plot - result
figure(1)
subplot(3,2,1)
step(G_WC, t)
title('Reference Tracking')
legend('Without Controler')
grid on

subplot(3,2,2)
step(G_1, t)
title('Reference Tracking')
legend('1-DOF PD')
grid on

subplot(3,2,3)
step(G_2, t)
title('Reference Tracking')
legend('2-DOF PID')
grid on

subplot(3,2,4)
step(G_t_pidZN_1, t)
title('Reference Tracking')
legend('Ziegler–Nichols Method 1')
grid on

subplot(3,2,5)
step(GA_PID_C_ITAE, t)
title('Reference Tracking')
legend('GA PID with FF ITAE')
grid on

subplot(3,2,6)
step(GA_PID_C_IAE, t)
title('Reference Tracking')
legend('GA PID with FF IAE')
grid on

figure(2)
step(G_WC, G_1, G_2, G_t_pidZN_1, GA_PID_C_ITAE,GA_PID_C_IAE, t)
title('Reference Tracking')
legend('Without Controler','1-DOF PD', '2-DOF PID', 'Ziegler–Nichols Method 1', 'GA PID with FF ITAE', 'GA PID with FF IAE', 'GA_PID_C_IAE')
grid on


