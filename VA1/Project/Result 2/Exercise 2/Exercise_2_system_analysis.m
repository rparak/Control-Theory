%% Transfer function coefficients 

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
H_s = minreal(H_s);

%% Step response 

figure(1)
step(H_s);
xlabel('time'); 
ylabel('H_s');
title('Step response without any feedback'); 
info = stepinfo(H_s)

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

