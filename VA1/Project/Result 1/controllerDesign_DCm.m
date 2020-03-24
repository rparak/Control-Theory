%*********************** Program initialization ************************%
clear all;
close all;
clc;

%************************ Specified parameters *************************%
J = 3.2284e-6;
b = 3.5077e-6;
K = 0.0274;
R = 4;
L = 2.75e-6;

%% ************** Position characteristics and regulation ************ %%
%************************ Systems declaration **************************%
s = tf('s');
G_sPos = K/(J*L*s^3+J*R*s^2+L*b*s^2+b*R*s+s*K^2);

%************************ Step characteristic **************************%
figure('Name','DC motors position characteristics and regulation','NumberTitle','off');
subplot(3,3,1)
step(G_sPos, 'r');

title('Step response');
ylabel('h(t)');
grid on;

%*********************** Impulse characteristic ************************%
subplot(3,3,2)
impulse(G_sPos,'k');

title('Impulse response');
ylabel('g(t)');
grid on;

%********************** Frequency characteristic ***********************%
subplot(3,3,3)
nyquist(G_sPos,'b');

title('Frequency response');
xlabel('Re');
ylabel('Im');
grid on;

%************************ 2-DOF, 1-DOF methods *************************%
% pidtool
wb = 50; % proposed bandwith 50 rad/s from pidtool
PID_2 = pidtune(G_sPos,'PID2',wb);
G_DOF2 = tf(PID_2);

G_PID21 = G_DOF2(1);
G_PID22 = G_DOF2(2);
G_2Pos = G_PID21*feedback(G_sPos,G_PID22,1);

PID_1 = pidtune(G_sPos,'PID',wb);
G_1Pos   = feedback(G_sPos*PID_1,1);

subplot(3,3,4);
step(G_1Pos, 'g',G_2Pos, 'm');
title('2-DOF, 1-DOF methods');
legend('1-DOF','2-DOF');
grid on;

%************** Ziegler–Nichols computationally method *****************%
%P controller parameter
r0 = 20299.68;
P_3 = r0;

G_3Pos = feedback(P_3*G_sPos,1);

subplot(3,3,5);
step(G_3Pos,'y');
title('Ziegler–Nichols computationally method');
grid on;

%*************************** pidTuner method ***************************%
% pidTuner(G_sPos)
% Constants gained from pidTuner
Kp = 16.13;
Kd = 0.2734;

PD_4 = Kp + Kd*s;
G_4Pos = feedback(G_sPos*PD_4,1);

subplot(3,3,6);
step(G_4Pos,'c');
title('pidTuner method');
grid on;

%************************* All methods in one **************************%
subplot(3,3,[7 8 9]);
step(G_1Pos, 'g', G_2Pos, 'm', G_4Pos, 'c');
title('All methods in one except ZNc');
legend('1-DOF','2-DOF','pidTuner');
grid on;

%% ************** Speed characteristics and regulation *************** %%
%************************ Systems declaration **************************%
s = tf('s');
G_sSpd = K/(J*L*s^2+J*R*s+L*b*s+b*R+K^2);

%************************ Step characteristic **************************%
figure('Name','DC motors speed characteristics and regulation','NumberTitle','off');
subplot(3,2,1)
step(G_sSpd, 'r');

title('Step response');
ylabel('h(t)');
grid on;

%*********************** Impulse characteristic ************************%
subplot(3,2,2)
impulse(G_sSpd,'k');

title('Impulse response');
ylabel('g(t)');
grid on;

%********************** Frequency characteristic ***********************%
subplot(3,2,3)
nyquist(G_sSpd,'b');

title('Frequency response');
xlabel('Re');
ylabel('Im');
grid on;

%************************ 2-DOF, 1-DOF methods *************************%
% pidtool
wb = 50; % proposed bandwith 50 rad/s from pidtool
PID_2 = pidtune(G_sSpd,'PID2',wb);
G_DOF2 = tf(PID_2);

G_PID21 = G_DOF2(1);
G_PID22 = G_DOF2(2);
G_2Spd = G_PID21*feedback(G_sSpd,G_PID22,1);

PID_1 = pidtune(G_sSpd,'PID',wb);
G_1Spd   = feedback(G_sSpd*PID_1,1);

subplot(3,2,4);
step(G_1Spd, 'g',G_2Spd, 'm');
title('2-DOF, 1-DOF methods');
legend('1-DOF','2-DOF');
grid on;

%*************************** pidTuner method ***************************%
% pidTuner(G_sSpd)
% Constants gained from pidTuner
Kp = 185;

P_3 = Kp;
G_3Spd = feedback(G_sSpd*P_3,1);

subplot(3,2,5);
step(G_3Spd,'c');
title('pidTuner method');
grid on;

%************************* All methods in one **************************%
subplot(3,2,6);
step(G_1Spd, 'g', G_2Spd, 'm', G_3Spd, 'c');
title('All methods in one');
legend('1-DOF','2-DOF','pidTuner');
grid on;
