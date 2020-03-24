%% Genetic algorithm for exercise 3 speed
clear all 
close all 
clc

% Given values 

J = 3.2284*10^(-6); 
b = 3.5077*10^(-6); 
K = 0.0274; 
R = 4 ; 
L = 2.75*10^(-6); 

% For the transfer function 

p0 = K ; 
r0 = R*b+K^2; 
r1 = b*L+R*J; 
r2 = J*L;

%% Transfer function 
 
s = tf('s'); 
H_s = p0/(r0 + r1*s + r2*s^2); 
H_s = minreal(H_s)

%% Step response 

subplot(2, 1, 1);
step(H_s); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 

%% Genetic algorithm - cf function

s = tf('s');  
Kp = 28.6761282626275; 
Ki = 99.92914252054443;
Kd = 0.2134224134067444;

pid_c = Kp + Ki/s + Kd*s;

subplot(2, 1, 2); 
step(feedback(pid_c*H_s, 1)); 
xlabel('time'); 
ylabel('H_s');
title('Step response PID designed by genetic algorithm'); 

figure(2)
step(feedback(pid_c*H_s, 1)); 
xlabel('time'); 
ylabel('H_s');
title('Step response PID designed by genetic algorithm'); 