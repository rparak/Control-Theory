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

subplot(2, 1, 1);
step(H_s); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 

%% Find the good shape 

p_cara = [1 1.455*10^6 8.614*10^7]; 
racine = roots(p_cara); 
a = racine(1); 
b = racine(2);
K1 = (3.086*10^9)/(a*b);
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
title('Step response PID designed by dynamic inversion design'); 

figure(2) 
step(G_FB_PID, 'g');
xlabel('time'); 
ylabel('H_s');
title('Step response PID designed by dynamic inversion design'); 
