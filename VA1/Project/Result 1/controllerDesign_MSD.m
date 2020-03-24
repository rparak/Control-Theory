%*********************** Program initialization ************************%
clear all;
close all;
clc;

%************************ Specified parameters *************************%
m1 = 1.35;
m2 = 1.45;
k1 = 0.95;
k2 = 1.05;
b = 0.25;

%************************* System declaration **************************%
s = tf('s');
G_s = (k1*b*s+k1*k2)/(m1*m2*s^4+b*m2*s^3+b*m1*s^3+k1*m2*s^2+k2*m2*s^2+k2*m1*s^2+k1*b*s+k1*k2);

%************************* Step characteristic *************************%
figure('Name','Mass-Sping-Damper characteristics and regulation','NumberTitle','off');
subplot(3,2,1)
step(G_s, 'r');

title('Step response');
xlabel('Time t');
ylabel('h(t)');
grid on;

%*********************** Impulse characteristic ************************%
subplot(3,2,2)
impulse(G_s, 'k');

title('Impulse response');
xlabel('Time t');
ylabel('g(t)');
grid on;

%********************** Frequency characteristic ***********************%
subplot(3,2,3)
nyquist(G_s);

title('Frequency response');
xlabel('Re');
ylabel('Im');
grid on;

%******************* Genetic algorithm ITAE method *********************%
% Constants gained from function itae_ga
Kp = 0.011993911457062;
Ki = 0.207947383686235;
Kd = 0.781547855474085;

% Controller declaration
PID_1 = Kp + Ki/s + Kd*s;

% Circuit assembly
G_1 = feedback(PID_1*G_s,1);

subplot(3,2,4);
step(G_1,'g');
title('Genetic algorithm ITAE method');
grid on;

%************************** pidTuner method ***************************%
% pidTuner(G_s)
% Constants gained from pidTuner
Kp = 0.3111;
Ki = 0.03847;
Kd = 0.4676;

PID_2 = Kp + Ki/s + Kd*s;
G_2 = feedback(G_s*PID_2,1);

subplot(3,2,5);
step(G_2,'m');
title('pidTuner method');
grid on;

%************************* All methods in one **************************%
subplot(3,2,6);
step(G_1, 'g', G_2, 'm');
title('All methods in one');
legend('GA ITAE','pidTuner');
grid on;