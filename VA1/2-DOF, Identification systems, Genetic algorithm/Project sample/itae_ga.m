function [J_ITAE] = itae_ga(x)
    
    s = tf('s');
    %% System transfer
    G_s = (6*s + 3)/(0.5*s^3 + 4*s^2 + 0.5*s + 2);
    
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