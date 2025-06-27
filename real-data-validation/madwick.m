%% plot_circle_madgwick.m (IMU trajectory estimation using Madgwick filter)

filename = 'sensor-data/circle.csv';
T = readtable(filename);

%–– Time vector in seconds
time_str = string(T.Time);
t_dt     = datetime(time_str, 'InputFormat', 'HH:mm:ss.SSS');
t0       = t_dt(1);
t        = seconds(t_dt - t0);

%–– Raw IMU data
acc_raw  = [T.AccelX, T.AccelY, (T.AccelZ + 9.81)]';         % 3×N, in m/s^2
gyro_raw = [T.GyroX, T.GyroY, T.GyroZ]';   % 3×N, now in rad/s

%–– Adjust orientation of sensor axes if needed
acc_raw(1:2,:)  = acc_raw(1:2,:);  % flip X, Y
gyro_raw(1:2,:) = gyro_raw(1:2,:); % match acc flip

%–– Initialize state
N    = numel(t);
dt   = mean(diff(t));   % assuming uniform sampling

madgwick = MadgwickAHRS('SamplePeriod', dt, 'Beta', 0.1);  % tune Beta if needed

q     = zeros(4, N);   q(:,1) = [1; 0; 0; 0];
vel   = zeros(3,N);
pos   = zeros(3,N);

for k = 2:N
    acc_k  = acc_raw(:,k);
    gyro_k = gyro_raw(:,k);

    %–– Update orientation estimate
    madgwick.SamplePeriod = t(k) - t(k-1);  % handle small variations
    madgwick.UpdateIMU(gyro_k, acc_k);
    q(:,k) = madgwick.Quaternion';

    %–– Rotate body-frame acceleration to world frame
    Rwb = quat2rotm(q(:,k)');   % body to world
    acc_world = Rwb * acc_k;

    %–– Subtract gravity (in world frame)
    aw = acc_world - [0; 0; 0];

    %–– Integrate velocity & position
    vel(:,k) = vel(:,k-1) + aw * (t(k) - t(k-1));
    pos(:,k) = pos(:,k-1) + vel(:,k-1) * (t(k) - t(k-1)) + 0.5 * aw * (t(k) - t(k-1))^2;
end

%–– Plot 2D Trajectory
[~, name, ext] = fileparts(filename);
title2D = sprintf('Estimated Trajectory from %s%s', name, ext);

figure;
plot(pos(1,:), pos(2,:), '-o','LineWidth',1.5);
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;
title(title2D);

%–– Plot 3D Trajectory
title3D = sprintf('3D Estimated Trajectory from %s%s', name, ext);
figure;
plot3(pos(1,:), pos(2,:), pos(3,:), '-o','LineWidth',1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal; grid on;
view(45,20);
title(title3D);
