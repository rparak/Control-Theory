function Tw = calc_tw(G_s)
    s = tf('s');
    
    dt = 0.01;
    t  = 0:dt:10;
    y  = step(G_s,t);

    dy = diff(y)/dt; % derivation
    [m,idx] = max(dy);
    yi = y(idx);
    ti = t(idx);

    Kp = y(end); % gain
    Tu = ti - yi/m; % time delay
    Tn = (Kp-yi)/m+ti-Tu; % time constant

    Tw = Tn;
    %plot(t,y,'b--',[Tu Tu+Tn Tu+Tn],[0 Kp 0])
    %grid on
end