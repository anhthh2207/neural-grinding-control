
clc; clear all;

run("config.m")
Kp_t=9.6721; Ki_t=6.3267; Kd_t=0.0000;
Kp_ga=10.785; Ki_ga=0.571; Kd_ga=0.238;
Kp_rl=8.0356; Ki_rl=0.0188; Kd_rl=0;



C_ga = pid(Kp_ga, Ki_ga, Kd_ga);
C_t = pid(Kp_t, Ki_t, Kd_t);
C_rl = pid(Kp_rl, Ki_rl, Kd_rl); 


% time
t = 0:0.01:10;
fs = 1 / (t(2) - t(1)); 

% random impulse signal
rand_impulse_signal = zeros(size(t));
n_spikes = 10;                                 
min_width = round(0.4 * fs);                   
max_width = round(0.6 * fs);

for k = 1:n_spikes
    width = randi([min_width, max_width]);
    idx = randi([1, length(t) - width]); 
    rand_impulse_signal(idx : idx + width - 1) = rand() * (1.5 + rand());
end

% piecewise step signal
step_ladder = zeros(size(t));
step_ladder(t > 2) = 2;
step_ladder(t > 4) = -3;
step_ladder(t > 6) = 4;
step_ladder(t > 8) = -1;

% frequency for periodic signals
f = 0.25; % Hz 

% define signals
signals = {
    'Step',             ones(size(t));
    'Impulse',          rand_impulse_signal;
    'Sawtooth',         5 * sawtooth(2*pi*f*t);
    'Sine',             5 * sin(2*pi*f*t);
    'ExpSine',          exp(-0.2 * t) .* sin(2*pi*0.5*t);
    'RandomImpulseNoise', rand_impulse_signal;
    'PiecewiseStep',    step_ladder;
    'Composite',        5 * sin(t) + 8 * sin(3*t + 2) + 0.2 * t .* exp(2 * cos(4*t))
};


for i = 1:size(signals,1)
    signal_name = signals{i,1};
    r = signals{i,2};
    
    % closed-loop transfer functions
    T_ga = feedback(C_ga*G, 1);
    T_t  = feedback(C_t *G, 1);
    T_rl = feedback(C_rl*G, 1);
    
    % response
    y_ga = lsim(T_ga, r, t);
    y_t  = lsim(T_t,  r, t);
    y_rl = lsim(T_rl, r, t);
    
    % plot
    figure('Name', signal_name, 'NumberTitle', 'off', 'Position', [100, 100, 800, 600])
    plot(t, r, 'k--', 'LineWidth', 2); hold on;
    plot(t, y_t,  'b', 'LineWidth', 2);
    plot(t, y_ga, 'r', 'LineWidth', 2);
    plot(t, y_rl, 'g', 'LineWidth', 2);
    legend('Reference', 'GB PID', 'GA PID', 'RL PID', 'Location', 'Best', 'FontSize', 14);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Output', 'FontSize', 14);
    grid on;
    set(gca, 'FontSize', 14);
    
    % MSE
    mse_t  = mean((r - y_t').^2);
    mse_ga = mean((r - y_ga').^2);
    mse_rl = mean((r - y_rl').^2);

    fprintf('\n%s Input:\n', signal_name)
    fprintf('MSE (GB PID)      : %.4f\n', mse_t);
    fprintf('MSE (GA PID)      : %.4f\n', mse_ga);
    fprintf('MSE (RL PID)      : %.4f\n', mse_rl);
end
