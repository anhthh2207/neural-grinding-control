clc; clear;

%% problem setup
num1 = -20.15;
num0 = 550.5;
de2 = 1;
de1 = 36.7;
de0 = 564.5;
rg = 0.005;
ke = 3.0 * 1e3;
ks = 1.5 * 1e3;
cf = 0.01 * 1e3;
M = 2.9;

speed_loop = tf([num1, num0], [de2, de1, de0]);
rgOverS = tf(rg, [1, 0]);
end_effector = tf([ke*cf, ke*ks], [M, cf, (ke + ks)]);
G = speed_loop * rgOverS * end_effector;

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
title('Step Response with Optimized PID');
xlabel('Time (s)');
ylabel('Output');