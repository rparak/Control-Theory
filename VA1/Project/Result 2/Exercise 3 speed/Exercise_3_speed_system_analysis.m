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
r0 = R*b+K^2; 
r1 = b*L+R*J; 
r2 = J*L;

%% Transfer function 
 
s = tf('s'); 
H_s = p0/(r0 + r1*s + r2*s^2); 
H_s = minreal(H_s)
stepinfo(H_s)

%% Step response 

figure(1)
step(H_s); 
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 

%% Frequency response 

figure(2)
bode(H_s);

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