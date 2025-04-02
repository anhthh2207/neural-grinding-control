% parameters for dynamical model
kf = 0.1;

num1 = -20.15;
num0 = 550.5;
de2 = 1;
de1 = 36.7;
de0 = 564.5;

rg = 0.005;

ke = 3.0 * 1e3;
ks = [0.5, 1.5, 6.0]* 1e3;
cf = 0.01 * 1e3;

M = 2.9;


% trasfer functions of the system
speed_loop = tf([num1, num0], [de2, de1, de0]);
rgOverS = tf(rg, [1, 0]);


colors = ['r', 'b', 'g'];


% bode plot for the end effector
figure();
end_effector = [];

for i = 1:length(ks)
    end_effector = [end_effector, tf([ke*cf, ke*ks(i)], [M, cf, (ke + ks(i))])];
end

bodeplot(end_effector(1), 'r', end_effector(2), 'b' , end_effector(3), 'g');
legend('ks = 0.5N/mm','ks = 1.5N/mm', 'ks = 6N/mm');
title('Bode Plot of End Effector');
grid on;
xlabel('Frequency (Hz)');
saveas(gcf, 'assets/bode_end_effector.png');


% Nyquist diagram for open-loop system
figure();
G = [];

for i = 1:length(ks)
    G = [G, speed_loop * rgOverS * end_effector(i)];
end

nyquist(G(1), 'r', G(2), 'b' , G(3), 'g');
legend('ks = 0.5N/mm','ks = 1.5N/mm', 'ks = 6N/mm');
title('Nyquist Diagram of the Open-Loop System');
grid on;
xlabel('Frequency (Hz)');
saveas(gcf, 'assets/nyquist_open_loop.png')


% bode plot for the open-loop system
bodeplot(G(1), 'r', G(2), 'b' , G(3), 'g');
legend('ks = 0.5N/mm','ks = 1.5N/mm', 'ks = 6N/mm');
title('Bode Plot of Open-loop system');
grid on;
xlabel('Frequency (Hz)');
saveas(gcf, 'assets/bode_open_loop.png');










