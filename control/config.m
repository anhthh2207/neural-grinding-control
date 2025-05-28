% % parameters for dynamical model
% kf = 0.1;
% 
% num1 = -20.15;
% num0 = 550.5;
% de2 = 1;
% de1 = 36.7;
% de0 = 564.5;
% 
% rg = 0.005;
% 
% ke = 3.0 * 1e3;
% ks = 1.5 * 1e3;
% cf = 0.01 * 1e3;
% % ke = 3.0;
% % ks = 1.5;
% % cf = 0.01;
% M = 2.9;
% 
% 
% % define transfer function for the system
% speed_loop = tf([num1, num0], [de2, de1, de0]);
% rgOverS = tf(rg, [1, 0]);
% end_effector = tf([ke*cf, ke*ks], [M, cf, (ke + ks)]);
% 
% G = speed_loop * rgOverS * end_effector;






% parameters for dynamical model
% kf is not measured, but manually tuned based on performance
% Start with 0, 0.1 here is a safe range. Will need to tune further when
% operating on robot 	(unitless or m/N·s)
kf = 0.1;

%% 
num1 = -20.15; %How the rate of change of command affects speed (m/s² per input unit)
num0 = 550.5; %How the magnitude of input affects speed (gain) (m/s per input unit)
de2 = 1; % Always 1 for normalization (s²)
de1 = 36.7; % 	Damping-like behavior — affects how quickly the speed settles (s)
de0 = 564.5; % 	Inertia + control bandwidth — affects overshoot and natural frequency (unitless)

% Converts motor rotation into linear motion.  we measure the distance of the nut move when THE SCREW move one rotation

% Mark the nut’s position on the lead screw.
% Rotate the motor 1 full revolution manually or via software.
% Measure the linear distance moved → that's rg. (m/rev)
% rg = 0.005;
rg = 0.002; % ACTUAL NUMBER

% TOOL STIFFNESS. How the tool itself compreses under graiding force.
% Appear in the forcce model, as tool displacement. 
% To measure, press tooll into wall, and measure F/delta_X (N/m)
% ke = 3.0 * 1e3;

ke = 3.0 * 1e3; %base don paper, metal wheel are 45.6

%SPRING STIFFNESS -> The stiffness of the series spring between motor and tool.
% Hang known weight F -> measure compression delta x -> ks = F/delta_x (N/m)
% ks = 1.5 * 1e3;

ks = 0.25 * 1e3; % actual number

% Damping from spring friction and material. (N·s/m). Paper dont say how to
% measure
cf = 0.01 * 1e3;


% ke = 3.0;
% ks = 1.5;
% cf = 0.01;

% Weight the whole end-effector (kg)

% M = 2.9;

M = 1.65; %actual number

% define transfer function for the system
speed_loop = tf([num1, num0], [de2, de1, de0]);
rgOverS = tf(rg, [1, 0]);
end_effector = tf([ke*cf, ke*ks], [M, cf, (ke + ks)]);

G = speed_loop * rgOverS * end_effector;