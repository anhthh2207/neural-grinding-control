%% problem setup
run('../config.m')

%% optimiation problem setup
lb = [0 0 0];
ub = [1000 1000 1000];

opts = optimoptions('ga','Display','iter','PopulationSize',30,'MaxGenerations',40);
[x, fval] = ga(@(x) pid_cost(x, G), 3, [], [], [], [], lb, ub, [], opts);

Kp_opt = x(1);
Ki_opt = x(2);
Kd_opt = x(3);

fprintf('Optimized PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', Kp_opt, Ki_opt, Kd_opt);


%% evaluation 
C = pid(Kp_opt, Ki_opt, Kd_opt);
T = feedback(C*G, 1);

figure;
step(T, 0:0.01:5);
title('Step Response with GA optimized PID');
xlabel('Time (s)');
ylabel('Output');

Kp_ga = Kp_opt;
Ki_ga = Ki_opt;
Kd_ga = Kd_opt;