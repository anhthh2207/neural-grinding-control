function J = pid_cost(pid_params, G)
    Kp = pid_params(1);
    Ki = pid_params(2);
    Kd = pid_params(3);

    % PID controller
    C = pid(Kp, Ki, Kd);

    T = feedback(C*G, 1);

    t = 0:0.01:5;
    [y, ~] = step(T, t);
    e = 1 - y; 

    J = sum(e.^2)*0.01;
    
    % penalty for excessive overshoot
    if max(y) > 1.2 
        J = J + 100 * (max(y) - 1.2);
    end
end