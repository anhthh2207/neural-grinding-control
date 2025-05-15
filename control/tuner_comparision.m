clc; clear all;

run("pid_tuner.m");
run("pid_ga.m");

C_ga = pid(Kp_ga, Ki_ga, Kd_ga);
C_t = pid(Kp_t, Ki_t, Kd_t);

t = 0:0.01:10;
% r = 0.5*t + 0.8*sin(2*pi*0.2*t); 
% r(t>5) = r(t>5) + 2;     
r = 5 * sin(t) + 8 * sin(3*t + 2) + 0.2 * t .* exp(2 * cos(4*t));

T_ga = feedback(C_ga*G, 1);
T_t  = feedback(C_t*G, 1);

y_ga = lsim(T_ga, r, t);
y_t  = lsim(T_t,  r, t);

figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_ga, 'r', 'LineWidth', 2);
plot(t, y_t,  'b', 'LineWidth', 2);
legend('Reference (arbitrary)', 'GA PID', 'MATLAB PID', 'Location', 'Best');
xlabel('x (m)');
ylabel('Force (N)');
title('Comparison of GA-tuned and MATLAB PID Controllers');
grid on;

mse_ga = mean((r - y_ga').^2);
mse_t  = mean((r - y_t').^2);

fprintf('MSE (GA PID)      : %.4f\n', mse_ga);
fprintf('MSE (MATLAB PID)  : %.4f\n', mse_t);