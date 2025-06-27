%% plot_circle.m  (no ADC→m/s² conversion; dataset already in SI)
filename = 'sensor-data/triangle.csv';
T = readtable(filename);

%–– Time → seconds
time_str = string(T.Time);            
t_dt     = datetime(time_str,'InputFormat','HH:mm:ss.SSS');
t0       = t_dt(1);
t        = seconds(t_dt - t0);       % seconds since start

%–– Raw (already physical) IMU data
acc_raw  = [T.AccelX, T.AccelY, T.AccelZ + 9.81]';   % 3×N, in m/s^2
acc_raw(1:2,:) = -acc_raw(1:2,:);   
gyro_raw = [T.GyroX, T.GyroY, T.GyroZ]';   % 3×N, in °/s
% Now map to [Wx; Wy; Wz]
% gyro_raw = [ gyro_raw(2,:);  % Wx
%              gyro_raw(3,:);  % Wy
%              gyro_raw(1,:)]; % Wz


N   = numel(t);
acc = zeros(3,N);
omega = ones(3,N);
q = zeros(4,N);    q(:,1) = [1;0;0;0];
vel = zeros(3,N);
pos = zeros(3,N);

for k = 1:N
    % 1) Use acc_raw directly
    acc(:,k) = acc_raw(:,k);                

    % 2) Convert gyro to rad/s
    omega(:,k) = gyro_raw(:,k);  

    if k>1
        dt = t(k) - t(k-1);

        % 3) Quaternion propagation: q̇ = 0.5*Ω(ω)*q
        qk = q(:,k-1);
        W  = [    0,        -omega(:,k)';
              omega(:,k),  -skew(omega(:,k)) ];
        q(:,k) = qk + 0.5 * W * qk * dt;
        q(:,k) = q(:,k) / norm(q(:,k));

        % 4) Rotate accel into world frame & subtract gravity
        Rwb = quat2rotm(q(:,k)');           
        aw  = Rwb*acc(:,k) + [0;0;0];   

        % 5) Integrate velocity & position
        vel(:,k) = vel(:,k-1) + aw*dt;
        pos(:,k) = pos(:,k-1) + vel(:,k-1)*dt + 0.5*aw*dt^2;
    end
end

%–– Plot X–Y
%–– Later in plotting, extract just the name+extension
[~, name, ext] = fileparts(filename);  % name = 'square', ext = '.csv'
% Build title strings
title2D = sprintf('Estimated Trajectory from %s%s', name, ext);
title3D = sprintf('3D Estimated Trajectory from %s%s', name, ext);

figure;
plot(pos(1,:), pos(2,:), '-o','LineWidth',1.5);
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;
title(title2D);


% 3-D trajectory
figure;
plot3(pos(1,:), pos(2,:), pos(3,:), '-o','LineWidth',1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal; grid on;
view(45,20);    % azimuth=45°, elevation=20°
title(title3D);

%% helper: skew-symmetric for cross product
function S = skew(v)
    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0 ];
end
