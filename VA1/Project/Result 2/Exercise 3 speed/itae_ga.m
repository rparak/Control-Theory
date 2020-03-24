function [J_ITAE] = itae_ga(x)
    
    s = tf('s');
    %% System transfer
    G_s   = (3.086*10^9)/(s^2+1.455*10^6 + 8.614*10^7);
    
    %% PID - param
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    
    pid_c = Kp + Ki/s + Kd*s;
    %% ITAE - calculation
    dt = 0.01;
    t  = 0:dt:50;
    
    %step(feedback(pid_c*G_sys,1));
    e  = 1 - step(feedback(pid_c*G_s,1),t);
    %step(feedback(pid_c*G_s,1))
    
    J_ITAE  = sum(t'.*abs(e)*dt);
end