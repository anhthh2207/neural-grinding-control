clc; clear all;

run("config.m")

% Kp_t=9.6721; Ki_t=6.3267; Kd_t=0.0000;
% Kp_ga=7.677; Ki_ga=0.514; Kd_ga=0.239;
% Kp_rl=9.2672; Ki_rl=1.9768; Kd_rl=9.3521 * 1e-04;

Kp_t=9.6721; Ki_t=6.3267; Kd_t=0.0000;
Kp_ga=9.677; Ki_ga=0.478; Kd_ga=0.125;
Kp_rl=9.2672; Ki_rl=1.9768; Kd_rl=9.3521 * 1e-04;


C_ga = pid(Kp_ga, Ki_ga, Kd_ga);
C_t = pid(Kp_t, Ki_t, Kd_t);
C_rl = pid(Kp_rl, Ki_rl, Kd_rl); 

t = 0:0.01:10;
r = 5 * sin(t) + 8 * sin(3*t + 2) + 0.2 * t .* exp(2 * cos(4*t));


%% Response to sinusoidal waveform
T_ga = feedback(C_ga*G, 1);
T_t  = feedback(C_t*G, 1);
T_rl = feedback(C_rl * G, 1);  

y_ga = lsim(T_ga, r, t);
y_t  = lsim(T_t,  r, t);
y_rl = lsim(T_rl, r, t);   

figure('Units', 'pixels', 'Position', [100, 100, 800, 800]);  
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_t,  'b', 'LineWidth', 2);
plot(t, y_ga, 'r', 'LineWidth', 2);
plot(t, y_rl, 'g', 'LineWidth', 2);
legend('Reference', 'GB PID', 'GA PID', 'RL PID', 'Location', 'Best', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 16);
ylabel('Force (N)', 'FontSize', 16);
% title('Comparison of GA, GB, and RL PID Tuners', 'FontSize', 16);
grid on;
set(gca, 'FontSize', 16);
axis square; 

mse_ga = mean((r - y_ga').^2);
mse_t  = mean((r - y_t').^2);
mse_rl = mean((r - y_rl').^2);

fprintf('Sinusoidal reference\n')
fprintf('MSE (GB PID)      : %.4f\n', mse_t);
fprintf('MSE (GA PID)      : %.4f\n', mse_ga);
fprintf('MSE (RL PID)      : %.4f\n', mse_rl);


%% Time-Domain Response Characteristics
r = 1;
t_sim = 10;
t_step = 0:0.01:t_sim;
step_input = ones(size(t_step));

% GB 
T_t_step = feedback(C_t * G, 1);
y_t_step = lsim(T_t_step, step_input, t_step);
info_t = stepinfo(y_t_step, t_step, r);
% ss_error_t = abs((r - y_t_step(end)) / r) * 100;
ss_error_t = abs((r - y_t_step(end)));
mse_t = mean((step_input - y_t_step').^2);
[GM_t, PM_t, Wcg_t, Wcp_t] = margin(C_t * G);
peak_time_t = info_t.PeakTime;

% GA 
T_ga_step = feedback(C_ga * G, 1);
y_ga_step = lsim(T_ga_step, step_input, t_step);
info_ga = stepinfo(y_ga_step, t_step, r);
% ss_error_ga = abs((r - y_ga_step(end)) / r) * 100;
ss_error_ga = abs((r - y_ga_step(end)));
mse_ga = mean((step_input - y_ga_step').^2);
[GM_ga, PM_ga, Wcg_ga, Wcp_ga] = margin(C_ga * G);
peak_time_ga = info_ga.PeakTime;

% RL 
T_rl_step = feedback(C_rl * G, 1);
y_rl_step = lsim(T_rl_step, step_input, t_step);
info_rl = stepinfo(y_rl_step, t_step, r);
% ss_error_rl = abs((r - y_rl_step(end)) / r) * 100;
ss_error_rl = abs((r - y_rl_step(end)));
mse_rl = mean((step_input - y_rl_step').^2);
[GM_rl, PM_rl, Wcg_rl, Wcp_rl] = margin(C_rl * G);
peak_time_rl = info_rl.PeakTime;

% results
fprintf('Step reference\n')
fprintf("\nGB PID Controller:\n");
fprintf("Rise Time       : %.4f s\n", info_t.RiseTime);
fprintf("Peak Time       : %.4f s\n", peak_time_t);
fprintf("Settling Time   : %.4f s\n", info_t.SettlingTime);
fprintf("Overshoot       : %.4f %%\n", info_t.Overshoot);
fprintf("Steady-State Err: %.4f\n", ss_error_t);
fprintf("MSE             : %.4f\n", mse_t);
fprintf("Gain Margin     : %.4f\n", 20*log10(GM_t));
fprintf("Phase Margin    : %.4f °\n", PM_t);

fprintf("\nGA PID Controller:\n");
fprintf("Rise Time       : %.4f s\n", info_ga.RiseTime);
fprintf("Peak Time       : %.4f s\n", peak_time_ga);
fprintf("Settling Time   : %.4f s\n", info_ga.SettlingTime);
fprintf("Overshoot       : %.4f %%\n", info_ga.Overshoot);
fprintf("Steady-State Err: %.4f\n", ss_error_ga);
fprintf("MSE             : %.4f\n", mse_ga);
fprintf("Gain Margin     : %.4f\n", 20*log10(GM_ga));
fprintf("Phase Margin    : %.4f °\n", PM_ga);

fprintf("\nRL PID Controller:\n");
fprintf("Rise Time       : %.4f s\n", info_rl.RiseTime);
fprintf("Peak Time       : %.4f s\n", peak_time_rl);
fprintf("Settling Time   : %.4f s\n", info_rl.SettlingTime);
fprintf("Overshoot       : %.2f %%\n", info_rl.Overshoot);
fprintf("Steady-State Err: %.4f\n", ss_error_rl);
fprintf("MSE             : %.4f\n", mse_rl);
fprintf("Gain Margin     : %.4f\n", 20*log10(GM_rl));
fprintf("Phase Margin    : %.4f °\n", PM_rl);


% Plot all in one figure
figure('Units', 'pixels', 'Position', [100, 100, 800, 800])
plot(t_step, step_input, 'k--', 'LineWidth', 2); hold on;
plot(t_step, y_t_step, 'b', 'LineWidth', 2);
plot(t_step, y_ga_step, 'r', 'LineWidth', 2);
plot(t_step, y_rl_step, 'g', 'LineWidth', 2);

legend('Reference', 'GB PID', 'GA PID', 'RL PID', 'Location', 'Best', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 16);
ylabel('Force (N)', 'FontSize', 16);
% title('Step Response: GB vs GA vs RL PID Controllers', 'FontSize', 16);
grid on;
set(gca, 'FontSize', 16);

