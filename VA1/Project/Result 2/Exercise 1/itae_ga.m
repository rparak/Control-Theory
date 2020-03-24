function [J_ITAE] = itae_ga(x)
    
    s = tf('s');
    %% System transfer
    G_s   = (0.1213*s+0.5096)/(s^4+0.3576*s^3+2.206*s^2+0.1213*s+0.5096);
    
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