clc; clear all;

%% problem setup
run('config.m')

%% optimiation problem setup
lb = [0 0 0];
ub = [1000 1000 1000];

options = optimoptions('particleswarm', 'Display', 'iter', 'SwarmSize', 30, 'MaxIterations', 40);
[x, fval] = particleswarm(@(x) pid_cost(x, G), 3, lb, ub, options);

Kp_opt = x(1);
Ki_opt = x(2);
Kd_opt = x(3);

fprintf('Optimized PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', Kp_opt, Ki_opt, Kd_opt);


%% evaluation 
C = pid(Kp_opt, Ki_opt, Kd_opt);
T = feedback(C*G, 1);

figure;
step(T, 0:0.01:5);
title('Step Response with Optimized PID');
xlabel('Time (s)');
ylabel('Output');