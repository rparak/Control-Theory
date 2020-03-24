%% Exercise no. 7 (PART - b) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 12.11.2018

%% Identification systems from unit step response
clc
clear all 
close all

%% Example no.1: Dead time
s = tf('s');

G_d = (3/(3*s + 1))*exp(-4*s);
step(G_d);
grid on

%% Example no.2: Approximation of the first derivative without dead time
G_without_DT1 = 2/(10*s+1);
G_without_DT2 = 2/(9.97*s + 1);

subplot(3,1,1)
step(G_without_DT1,'b-');
grid on
legend('original characteristic')
subplot(3,1,2)
step(G_without_DT2,'r:');
grid on
legend('approximation characteristic')
subplot(3,1,3)
step(G_without_DT1,'b-',G_without_DT2,'r:');
grid on
legend('original characteristic','approximation characteristic')

%% Example no.3: Approximation of the first derivative with dead time - A type
G_with_DT1_A = (2/(10*s+1))*exp(-5*s);
G_with_DT2_A = (2/(9.98*s+1))*exp(-5*s);

subplot(3,1,1)
step(G_with_DT1_A,'b-');
grid on
legend('original characteristic')
subplot(3,1,2)
step(G_with_DT2_A,'r:');
grid on
legend('approximation characteristic')
subplot(3,1,3)
step(G_with_DT1_A,'b-',G_with_DT2_A,'r:');
grid on
legend('original characteristic','approximation characteristic')

%% Example no.3: Approximation of the first derivative with dead time - B type
G_with_DT1_B = 2/(0.1*s^2 + 10*s +1);
G_with_DT2_B = (2/(9.99*s+1))*exp(-0.005*s);

subplot(3,1,1)
step(G_with_DT1_B,'b-');
grid on
legend('original characteristic')
subplot(3,1,2)
step(G_with_DT2_B,'r:');
grid on
legend('approximation characteristic')
subplot(3,1,3)
step(G_with_DT1_B,'b-',G_with_DT2_B,'r:');
grid on
legend('original characteristic','approximation characteristic')