function metrics = analyze_pid_response(Kp, Ki, Kd, r, t_sim, plant)
    % ANALYZE_PID_RESPONSE analyzes the response of a PID-controlled system.
    %
    % Inputs:
    %   Kp, Ki, Kd - PID gains
    %   r - Reference signal (scalar or vector)
    %   t_sim - Final simulation time in seconds
    %   plant - Transfer function (tf object) of the system
    %
    % Output:
    %   metrics - struct with RiseTime, PeakTime, SettlingTime, Overshoot

    t = linspace(0, t_sim, 1000);

    controller = pid(Kp, Ki, Kd);

    sys_cl = feedback(controller * plant, 1);

    % handle reference input
    if isscalar(r)
        r = r * ones(size(t));
    elseif length(r) ~= length(t)
        error('Reference signal r must be scalar or same length as time vector.');
    end

    % simulate response
    [y, ~] = lsim(sys_cl, r, t);

    % analyze step response characteristics relative to final value of r
    S = stepinfo(y, t, r(end));

    % output metrics
    metrics.RiseTime = S.RiseTime;
    metrics.PeakTime = S.PeakTime;
    metrics.SettlingTime = S.SettlingTime;
    metrics.Overshoot = S.Overshoot;

    % visualization
    figure;
    plot(t, r, 'k--', 'DisplayName', 'Reference');
    hold on;
    plot(t, y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Output');
    xlabel('Time (s)');
    ylabel('Response');
    title('Closed-Loop PID Response');
    legend;
    grid on;
end
