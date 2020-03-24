%% Transfer function coefficients

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
r0 = 0; 
r1 = R*b+K^2; 
r2 = b*L+R*J;
r3 = J*L; 

%% Transfer function 
 
s = tf('s'); 
H_s = p0/(r0 + r1*s + r2*s^2 + r3*s^3); 
H_s = minreal(H_s)
stepinfo(H_s)

%% Step response 

t = 0:0.001:0.2;
figure(1)
step(H_s,t); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 
info = stepinfo(H_s);

%% Frequency response 

figure(2)
bode(H_s)

%% Stability

p_H_s = pole(H_s); 
e = 0; 

for i = 1:length(p_H_s)
    if p_H_s(i) > 0
        disp('system is not stable'); 
        e = 1; 
    end
end

if e ~= 1
    disp('System is stable !'); 
end