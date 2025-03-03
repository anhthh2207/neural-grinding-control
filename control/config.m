% parameters for dynamical model
num1 = -20.15;
num0 = 550.5;
de2 = 1;
de1 = 36.7;
de0 = 564.5;

rg = 1.0;

ke = 1.0;
ks = 1.0;
cf = 1.0;
M = 1.0;


% tune PID controller
speedLoop = tf([num1, num0], [de2, de1, de0]);
rgOverS = tf(rg, [1, 0]);
endEffector = tf([ke*cf, ke*ks], [M, cf, (ke + ks)]);

G = speedLoop * rgOverS * endEffector;

C = pidtune(G, 'PID');