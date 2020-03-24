%% Genetic algorithm for exercise 1
clear all 
close all 
clc

% Given values 
m1 = 1.35; 
m2 = 1.45; 
k1 = 0.95; 
k2 = 1.05;
b = 0.25; 

% For the transfer function 
p0 = k1*k2 ; 
p1 = k1*b ;

r0 = k1*k2;
r1 = k1*b ;
%r2 = k2*m1+(k1+k2)*m2;
r2 = k1*m2+(m1+m2)*k2;
r3 = b*(m2+m1);
r4 = m1*m2;


%% Transfer function 
 
s = tf('s'); 
H_s = (p0 + p1*s)/(r0 + r1*s + r2*s^2 + r3*s^3 + r4*s^4); 
H_s = minreal(H_s)
%% Step response 

figure(1)
subplot(2, 1, 1); 
step(H_s); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 
info = stepinfo(H_s);

%% Genetic algorithm - cf function

s = tf('s');  
%Kp = 62.70496068279613; 
%Ki = 35.02596253689667;
%Kd = 99.99993894788008;

Kp =0.011841400639239663;
Ki = 0.20597135048710813; 
Kd = 0.7752899527549744; 

pid_c = Kp + Ki/s + Kd*s;

figure(1)
subplot(2, 1, 2); 
step(feedback(pid_c*H_s, 1)); 
xlabel('time'); 
ylabel('H_s');
title('Step response, PID designed by genetic algorithm'); 

figure(2)
step(feedback(pid_c*H_s, 1)); 
xlabel('time'); 
ylabel('H_s');
title('Step response, PID designed by genetic algorithm'); 