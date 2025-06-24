%% plot_circle_trajectory_full.m
% Full pipeline: load CSV → detect stationary → AHRS orientation → accel→world →
% zero-velocity compensation → integrate → 6DOF animation

clear; close all; clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

%% ------------------------------------------------------------------------
%% —– Dataset‐agnostic load & parse —–
T = readtable("Datasets\circle.csv");
% For circle.csv:
timeStr = string(T.Time);
t0      = datetime(timeStr(1),'InputFormat','HH:mm:ss.SSS');
time    = seconds(datetime(timeStr,'InputFormat','HH:mm:ss.SSS') - t0);

% For straight‐line file, if the Time is implicit:
% T.Properties.VariableNames = matlab.lang.makeValidName(T.Properties.VariableNames);
% time = T.TimeSeconds;  % or however they encode it
datasetType = 'circle';
% Compute samplePeriod
if strcmp(datasetType,'circle')
    samplePeriod = median(diff(time)); 
else
    samplePeriod = 1/256;
end

% Pull accel & gyro
if strcmp(datasetType,'circle')
    accX = T.AccelX; accY = T.AccelY; accZ = T.AccelZ;
    gyrX = T.GyroX;  gyrY = T.GyroY;  gyrZ = T.GyroZ;
else
    accX = T.AccelerometerX_ms2_;  % makeValidName version
    accY = T.AccelerometerY_ms2_;
    accZ = T.AccelerometerZ_ms2_;
    gyrX = T.GyroscopeX_deg_s_;
    gyrY = T.GyroscopeY_deg_s_;
    gyrZ = T.GyroscopeZ_deg_s_;
end

%% —– Filter normalizations —–
Fs = 1/samplePeriod;  Fn = Fs/2;
hpCutOff = 0.001;   lpCutOff = 5;   % you may lower lpCutOff << Fs/2 for circle

Wn_hp = hpCutOff / Fn;  
Wn_lp = lpCutOff / Fn;  

[b,a]   = butter(1, Wn_hp,'high');  acc_hp = abs(filtfilt(b,a, sqrt(accX.^2+accY.^2+accZ.^2) ));
% moving-average fallback
Fs       = 1/median(diff(time));
window   = max(1,round(0.5*Fs));
acc_lp   = movmean(acc_hp, window);
stationary = acc_lp < 0.05;

%% ------------------------------------------------------------------------
% 3) Compute orientation via Madgwick AHRS
N = numel(time);
quat = zeros(N,4);
AHRSalg = AHRS('SamplePeriod', samplePeriod, 'Kp',1, 'KpInit',1);

% 3.1) Initial convergence using mean accel over first 2 sec
initPeriod = 2;
idxInit = find(time <= initPeriod);
for i = 1:2000
    AHRSalg.UpdateIMU([0 0 0], [mean(accX(idxInit)), mean(accY(idxInit)), mean(accZ(idxInit))]);
end

% 3.2) Full update
for i = 1:N
    if stationary(i)
        AHRSalg.Kp = 0.5;
    else
        AHRSalg.Kp = 0;
    end
    AHRSalg.UpdateIMU( deg2rad([gyrX(i), gyrY(i), gyrZ(i)]), ...
                       [accX(i), accY(i), accZ(i)] );
    quat(i,:) = AHRSalg.Quaternion;
end

%% ------------------------------------------------------------------------
% 4) Rotate accel into Earth frame & remove gravity
%    quaternConj and quaternRotate are from 'Quaternions' folder
acc_earth = quaternRotate([accX, accY, accZ], quaternConj(quat));
acc_earth = acc_earth * 9.81;           % convert g→m/s^2

% 4.1) subtract gravity in Earth Z
acc_earth(:,3) = acc_earth(:,3) - 9.81;

%% ------------------------------------------------------------------------
% 5) Integrate to velocity with zero-velocity updates
vel = zeros(size(acc_earth));
for i = 2:N
    vel(i,:) = vel(i-1,:) + acc_earth(i,:) * samplePeriod;
    if stationary(i)
        vel(i,:) = [0 0 0];
    end
end

% 5.1) Remove drift on each non-stationary segment
velDrift = zeros(size(vel));
statStart = find([0; diff(stationary)] == -1);
statEnd   = find([0; diff(stationary)] ==  1);
for k = 1:numel(statEnd)
    len    = statEnd(k) - statStart(k);
    driftR = vel(statEnd(k)-1,:) / len;
    ramps  = (1:len)' * driftR;
    velDrift(statStart(k):statEnd(k)-1, :) = ramps;
end
vel = vel - velDrift;

%% ------------------------------------------------------------------------
% 6) Integrate to position
pos = zeros(size(vel));
for i = 2:N
    pos(i,:) = pos(i-1,:) + vel(i,:) * samplePeriod;
end

%% ------------------------------------------------------------------------
% 7) 6DOF Animation
%    quatern2rotMat converts N×4 quats → 3×3×N rotation matrices
R = quatern2rotMat(quat);  

% Optional: extend end pose to hold final frame
% extraTime = 20;

% extend final pose by extraTime seconds
extraTime = 20;               % seconds to hold final frame
Fs        = 1/samplePeriod;   % e.g. 256 Hz
nExtra    = round(extraTime * Fs);  % now guaranteed integer

pos = [ pos;     repmat(pos(end,:), nExtra, 1) ];
R   = cat(3, R, repmat(R(:,:,end), [1,1,nExtra]));
pos = [pos;     repmat(pos(end,:), nExtra, 1)];
R   = cat(3, R, repmat(R(:,:,end), [1,1,nExtra]));

SixDOFAnimation(pos, R, ...
    'SamplePlotFreq', 1, ...
    'Trail','All', ...
    'AxisLength',0.1, ...
    'ShowArrowHead',false, ...
    'Xlabel','X (m)', ...
    'Ylabel','Y (m)', ...
    'Zlabel','Z (m)', ...
    'ShowLegend',false, ...
    'CreateAVI',false);
