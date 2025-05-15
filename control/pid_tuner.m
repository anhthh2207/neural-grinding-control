run('config.m')

C = pidtune(G, 'PID');

fprintf('PID Tuner: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', C.Kp, C.Ki, C.Kd);

%% evaluation 
T = feedback(C*G, 1);

figure;
step(T, 0:0.01:5);
title('Step Response with Built-in Tuner PID');
xlabel('Time (s)');
ylabel('Output');

Kp_t = C.Kp;
Ki_t = C.Ki;
Kd_t = C.Kd;