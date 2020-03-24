function [J_ITAE] = itae_ga(x)
    
    M = 0.5;
    m = 0.2;
    b = 0.1;
    I = 0.006;
    g = 9.8;
    l = 0.3;
    q = (M+m)*(I+m*l^2)-(m*l)^2;

    s = tf('s');
    %% System transfer
    P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
    
    %% PID - param
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    
    pid_c = Kp + Ki/s + Kd*s;
    %% ITAE - calculation
    dt = 0.001;
    t  = 0:dt:10;
    
    e  = 1 - step(feedback(pid_c*P_pend,1),t);
    
    J_ITAE  = sum(t'.*abs(e)*dt);
end