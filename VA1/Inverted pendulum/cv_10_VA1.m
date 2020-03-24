%% Exercise no. 10 (PART - 1) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 28.11.2018

%% Inverted pendulum
clc
clear all
close all

%% Initialization parameters
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

%% System transfer
P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

%% 1. Engineering method
Kp = 100;
Ki = 1;
Kd = 20;

C = pid(Kp, Ki, Kd);

G_N1_e = feedback(P_pend, C);

G_N2_e = feedback(1, P_pend*C)*P_cart;

t=0:0.01:10;

impulse(G_N1_e, G_N2_e,t)
axis([0, 10, -0.3, 0.3]);

%% 2. Root-locus
rlocus(P_pend)

C = 1/s;
rlocus(C*P_pend)

zeros = zero(C*P_pend);
poles = pole(C*P_pend);

z = [-3 -4];
p = 0;
k = 1;
C = zpk(z,p,k);
rlocus(C*P_pend)

selected_point = -3.5367 + 0.7081i;
k              = 20.2396;
poles          = [0, -85.1333, -3.5232 + 0.7086i, -3.5232 - 0.7086i]; 

K = 20;
T = feedback(P_pend,K*C);
impulse(T)

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
T2 = feedback(1,P_pend*C)*P_cart;
t = 0:0.01:8.5;
impulse(T2, t);

RL_pid = pid(K*C);
%% 3. Genetic algorithm -> ITAE
Kp = 76.51;
Ki = 99.999;
Kd = 62.359;

C = pid(Kp, Ki, Kd);

G_N1_e = feedback(P_pend, C);

G_N2_e = feedback(1, P_pend*C)*P_cart;

t=0:0.01:10;

impulse(G_N1_e, G_N2_e,t)
axis([0, 10, -0.3, 0.3]);

%% 4. Genetic algorithm -> IAE
Kp = 41.057;
Ki = 99.999;
Kd = 4.521;

C = pid(Kp, Ki, Kd);

G_N1_e = feedback(P_pend, C);

G_N2_e = feedback(1, P_pend*C)*P_cart;

t=0:0.01:10;

impulse(G_N1_e, G_N2_e,t)
axis([0, 10, -0.3, 0.3]);

