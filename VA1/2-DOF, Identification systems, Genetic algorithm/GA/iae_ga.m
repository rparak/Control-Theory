function [J_IAE] = iae_ga(x)
    
    s = tf('s');
    %% System transfer
    G_s   = 1/(s*(s+1)*(s+2));
    
    %% PID - param
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    
    pid_c = Kp + Ki/s + Kd*s;
    
    %% IAE - calculation
    dt = 0.01;
    t  = 0:dt:50;
    
    %step(feedback(pid_c*G_sys,1));
    e  = 1 - step(feedback(pid_c*G_s,1),t);
    
    J_IAE  = sum((e.^2)*dt);
end