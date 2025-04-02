% parameters for dynamical model
kf = 0.1;

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


% tune PID controller
speed_loop = tf([num1, num0], [de2, de1, de0]);
rgOverS = tf(rg, [1, 0]);
end_effector = tf([ke*cf, ke*ks], [M, cf, (ke + ks)]);

G = speed_loop * rgOverS * end_effector;


% bode plot for the speed loop 
figure();
bode(end_effector); 
grid on;
title('Bode Plot of End Effector');
saveas(gcf, 'assets/bode_end_effector.png');


% bode plot for the open-loop system
figure();
bode(G); 
grid on;
title('Bode Plot of Open-loop system');
saveas(gcf, 'assets/bode_open_loop.png');


% Nyquist diagram 
figure();
nyquist(G);
grid on;
title('Nyquist Diagram of the Open-Loop System');
saveas(gcf, 'assets/nyquist_open_loop.png')








