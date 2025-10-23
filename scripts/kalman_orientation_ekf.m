function angle_out = kalman_orientation_ekf(ax, ay, az, gx, gy,dt)
%#codegen

% Persistent variables to store Kalman states
persistent x_roll x_pitch P_roll P_pitch

deg2rad = pi / 180;
%rad2deg = 180 / pi;

% Initialize on first call
if isempty(x_roll)
    % State: [angle; angular_rate]
    x_roll  = zeros(2,1);
    x_pitch = zeros(2,1);
    P_roll  = eye(2) * 0.1;
    P_pitch = eye(2) * 0.1;
end

% Measurement model: only angle is measured from accelerometer
H = [1 0];

% System model
A = [1 dt; 0 1];
Q = [1e-5 0; 0 1e-3];   % Process noise (tune for gyro trust)
R = 1e-2;               % Measurement noise (tune for accel trust)

% ==== ROLL AXIS ====

% Gyroscope input (rad/s)
gyro_roll = gy * deg2rad;

% Predict
x_roll = A * x_roll;
x_roll(2) = gyro_roll; % Update angular rate directly
P_roll = A * P_roll * A' + Q;

% Measurement from accelerometer (roll = atan2(ay, az))
z_roll = atan2(ay, az);

% Kalman Gain
K = P_roll * H' / (H * P_roll * H' + R);

% Update
x_roll = x_roll + K * (z_roll - H * x_roll);
P_roll = (eye(2) - K * H) * P_roll;

% ==== PITCH AXIS ====

gyro_pitch = gx * deg2rad;
x_pitch = A * x_pitch;
x_pitch(2) = gyro_pitch;
P_pitch = A * P_pitch * A' + Q;

% Measurement from accelerometer (pitch = atan(-ax / sqrt(ay^2 + az^2)))
z_pitch = atan(-ax / sqrt(ay^2 + az^2));

K = P_pitch * H' / (H * P_pitch * H' + R);
x_pitch = x_pitch + K * (z_pitch - H * x_pitch);
P_pitch = (eye(2) - K * H) * P_pitch;

% Output angles in degrees
angle_out = [x_roll(1); x_pitch(1)];% * rad2deg;
end


