clc; clear all;

run("config.m")
Kp_t=1.0105; Ki_t=0.5955; Kd_t=0.0170;
Kp_ga=1.453; Ki_ga=0.129; Kd_ga=0.052;
Kp_rl=0.8198; Ki_rl=1.0633; Kd_rl=0;


C_ga = pid(Kp_ga, Ki_ga, Kd_ga);
C_t = pid(Kp_t, Ki_t, Kd_t);
C_rl = pid(Kp_rl, Ki_rl, Kd_rl); 

t = 0:0.01:10;
% r = 0.5*t + 0.8*sin(2*pi*0.2*t); 
% r(t>5) = r(t>5) + 2;     
r = 5 * sin(t) + 8 * sin(3*t + 2) + 0.2 * t .* exp(2 * cos(4*t));


%% Response to sinusoidal waveform
T_ga = feedback(C_ga*G, 1);
T_t  = feedback(C_t*G, 1);
T_rl = feedback(C_rl * G, 1);  

y_ga = lsim(T_ga, r, t);
y_t  = lsim(T_t,  r, t);
y_rl = lsim(T_rl, r, t);   

figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_ga, 'r', 'LineWidth', 2);
plot(t, y_t,  'b', 'LineWidth', 2);
plot(t, y_rl, 'g', 'LineWidth', 2);
legend('Reference', 'GA PID', 'CST PID', 'RL PID', 'Location', 'Best');
xlabel('x (m)');
ylabel('Force (N)');
title('Comparison of GA, CST, and RL PID Tuners');
grid on;

mse_ga = mean((r - y_ga').^2);
mse_t  = mean((r - y_t').^2);
mse_rl = mean((r - y_rl').^2);

fprintf('MSE (GA PID)      : %.4f\n', mse_ga);
fprintf('MSE (MATLAB PID)  : %.4f\n', mse_t);
fprintf('MSE (RL PID)      : %.4f\n', mse_rl);


%% Time-Domain Response Characteristics
r = 1;
t_sim = 5;

metrics_t = analyze_pid_response(Kp_t, Ki_t, Kd_t, r, t_sim, G);
fprintf("CST:\n");
disp(metrics_t);

metrics_ga = analyze_pid_response(Kp_ga, Ki_ga, Kd_ga, r, t_sim, G);
fprintf("GA:\n");
disp(metrics_ga);


metrics_rl = analyze_pid_response(Kp_rl, Ki_rl, Kd_rl, r, t_sim, G);
fprintf("RL:\n");
disp(metrics_rl);

