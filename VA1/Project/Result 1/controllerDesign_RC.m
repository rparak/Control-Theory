%*********************** Program initialization ************************%
clear all;
close all;
clc;

%************************ Specified parameters *************************%
C1 = 1e-6;
C2 = 1.15e-6;
R1 = 5e5;
R2 = 0.5e4;

%************************* System declaration **************************%
s = tf('s');
G_s = 1/(R1*R2*C1*C2*s^2 + R1*C1*s + R1*C2*s + R2*C2*s + 1);

%************************ Step characteristic **************************%
figure('Name','RC circuit characteristics and regulation','NumberTitle','off');
subplot(3,3,1)
step(G_s, 'r');

title('Step response');
ylabel('h(t)');
grid on;

%*********************** Impulse characteristic ************************%
subplot(3,3,2)
impulse(G_s,'k');

title('Impulse response');
ylabel('g(t)');
grid on;

%********************** Frequency characteristic ***********************%
subplot(3,3,3)
nyquist(G_s,'b');

title('Frequency response');
xlabel('Re');
ylabel('Im');
grid on;

%************************ 2-DOF, 1-DOF methods *************************%
% pidtool
wb = 3.5; % proposed bandwith 3.5 rad/s from pidtool
PID_2 = pidtune(G_s,'PID2',wb);
G_DOF2 = tf(PID_2);

G_PID21 = G_DOF2(1);
G_PID22 = G_DOF2(2);
G_2 = G_PID21*feedback(G_s,G_PID22,1);

PID_1 = pidtune(G_s,'PID',wb);
G_1   = feedback(G_s*PID_1,1);

subplot(3,3,4);
step(G_1, 'g',G_2, 'm');
title('2-DOF, 1-DOF methods');
legend('1-DOF','2-DOF');
grid on;

%********************** Ziegler-Nichols 1 method ***********************%
dt = 0.01;
t  = 0:dt:10;
y  = step(G_s,t);

dy = diff(y)/dt;
[m,idx] = max(dy);
yi = y(idx);
ti = t(idx);

Kp = y(end);
Tu = ti - yi/m;
Tn = (Kp-yi)/m+ti-Tu;

subplot(3,3,5);
hold on;
plot(t,y,'b--',[Tu Tu+Tn Tu+Tn],[0 Kp 0],'k:');
grid on;

% PID parameters
r0 = 1.2*1/Kp*(Tn/Tu);
Ti = 2*Tu;
Td = 0.5*Tu;

PID_3 = r0*(1 + 1/(Ti*s) + Td*s);

G_3 = feedback(PID_3*G_s,1);

t = 0:0.01:10;
step(G_3,t,'y');
legend('Step response','Auxiliary lines', 'Regulated system');
title('Ziegler–Nichols 1 method');
grid on;

%*************************** pidTuner method ***************************%
% pidTuner(G_s)
% Constants gained from pidTuner
Kp = 5.598;
Ki = 4.769;
Kd = 0.08913;

PID_4 = Kp + Ki/s + Kd*s;
G_4 = feedback(G_s*PID_4,1);

subplot(3,3,6);
step(G_4,'c');
title('pidTuner method');
grid on;

%************************* All methods in one **************************%
subplot(3,3,[7 8 9]);
step(G_1, 'g', G_2, 'm', G_3, 'y', G_4, 'c');
title('All methods in one');
legend('1-DOF','2-DOF','Ziegler-Nichols','pidTuner');
grid on;