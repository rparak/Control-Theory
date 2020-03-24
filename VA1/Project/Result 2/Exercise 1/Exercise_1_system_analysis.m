%% Transfer function coefficients 

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
r2 = k1*m2+(m1+m2)*k2;
r3 = b*(m2+m1);
r4 = m1*m2;

%% Transfer function 

s = tf('s'); 
H_s = (p0 + p1*s)/(r0 + r1*s + r2*s^2 + r3*s^3 + r4*s^4); 
H_s = minreal(H_s);
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


