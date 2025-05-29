function J = pid_cost(pid_params, G)
    Kp = pid_params(1);
    Ki = pid_params(2);
    Kd = pid_params(3);

    C = pid(Kp, Ki, Kd);

    T = feedback(C*G, 1);

    t = 0:0.01:5;
    [y, ~] = step(T, t);
    e = 1 - y; 
    J = sum(e.^2) * 0.01;

    if max(y) > 1.2 
        J = J + 100 * (max(y) - 1.2);
    end

    L = C * G;
    [GM, PM, ~, ~] = margin(L);
    GM_dB = 20*log10(GM);

    if GM_dB < 10
        J = J + 1e5 * abs(GM_dB - 10); 
    end
    if PM < 45
        J = J + 1e5 * (45 - PM);  
    end
end
