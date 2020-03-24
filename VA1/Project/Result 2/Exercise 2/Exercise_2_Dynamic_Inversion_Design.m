%% Dynamic inversion design for exercise 2 

clear all 
close all 
clc

% Given values
C1 = 1*10^(-6);
C2 = 1.15*10^(-6);
R1 = 5*10^5;
R2 = 0.5*10^4; 

% For the transfer function
p0 = 1; 
r0 = 1; 
r1 = R1*C1+R1*C2+R2*C2; 
r2 = R1*R2*C1*C2; 

%% Transfer function 

s = tf('s'); 
H_s = p0/(r0 + r1*s + r2*s^2);
H_s = minreal(H_s)
subplot(2, 1, 1); 
step(H_s, 'r'); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 
hold on ; 

%% Find the good shape 

p_cara = [1 375.9 347.8]; 
racine = roots(p_cara); 
a = racine(1); 
b = racine(2);
K1 = 347.8/(a*b); 
T1 = -1/b; 
T2 = -1/a;
Tdelay = 0 ; %there is no delay
alpha = 1.282; %overshoot = 0,05
beta = 2.718; %overshoot = 0,05
Tw = calc_tw(H_s); 

%% PID Controller design
Ti = T1 + T2; 
r0 = (2*Ti)/(K1*2*Tw); 
Td = (T1*T2)/(T1+T2); 

G_pid = r0*(1+1/(Ti*s)+Td*s);
G_FB_PID = feedback(G_pid*H_s, 1); 
subplot(2, 1, 2); 
step(G_FB_PID, 'g'); 
xlabel('time'); 
ylabel('H_s');
title('Step response, PID designed by dynamic inversion design'); 

figure(2)
step(G_FB_PID, 'g'); 
xlabel('time'); 
ylabel('H_s');
title('Step response, PID designed by dynamic inversion design'); 



