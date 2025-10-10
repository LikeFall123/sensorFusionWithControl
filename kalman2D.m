function [tiltAngle,measRollPitch] = kalman2D(ax, ay, az, gx, gy,sampleTime,Q_kf,R_kf)
%#codegen

persistent x P instance

dt = single(sampleTime); % Sample time
deg2rad = single(pi / 180);

% Initialize persistent state vector and covariance matrix
if isempty(instance)
    % State: [theta_x; theta_y; omega_x; omega_y]
    x = single(zeros(4, 1));
    P = single(eye(4) * 0.1);
    instance = true;
end

% System dynamics matrix
A = single([1 0 dt 0;
             0 1 0 dt;
             0 0 1  0;
             0 0 0  1]);

% Process noise covariance (tune as needed)
Q = single(Q_kf);

% Measurement matrix: we only measure [theta_x, theta_y]
H = single([1 0 0 0;
            0 1 0 0]);

% Measurement noise covariance
R = single(R_kf);

% ====== PREDICT STEP ======

% Inject new gyro measurements into angular rate state (rad/s)!!!!!!!!!!!!
x(3) = gy * deg2rad;
x(4) = gx * deg2rad;

% Can be:
   % B = single([0 0;
   %              0 0;
   %              1 0;
   %              0 1]);
   % 
   %  % ===== PREDICT STEP =====
   %  % Gyro control input (rad/s)
   %  u = [gx; gy] * deg2rad;
   % 
   %  % Predict next state
   %  x = A * x + B * u;

% Predict state and covariance
x = A * x;
P = A * P * A' + Q;

% ====== MEASUREMENT FROM ACCELEROMETER ======

% Estimate roll and pitch from accelerometer
roll_meas  = -atan2(ax,az);
pitch_meas = -atan2(ay,az);
z = [roll_meas; pitch_meas];

measRollPitch = z;


% ====== UPDATE STEP ======
K = P * H' / (H * P * H' + R);
x = x + K * (z - H * x);
P = (eye(4) - K * H) * P;

% Output estimated [roll; pitch]
tiltAngle = x(1:2);
end