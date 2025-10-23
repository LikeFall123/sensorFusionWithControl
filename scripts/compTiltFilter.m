function [tiltAngle, measRollPitch] = compTiltFilter(ax, ay, az, gx_deg, gy_deg, sampleTime)
%#codegen


dt = single(sampleTime); % Sample time
deg2rad = single(pi / 180);
% rad2deg = single(180 / pi);

% defaults
fc = 10;

% alpha (gyro high-pass / accel low-pass) using time constant tau = 1/(2*pi*fc)
tau = 1 / (2*pi*fc);
alpha = tau / (tau + dt);   % typical complement: angle = alpha*(angle+gyro*dt) + (1-alpha)*accAngle

% Estimate roll and pitch from accelerometer
roll_meas  = atan2(ax,az);
pitch_meas = atan2(ay , sqrt(ax^2 + (az)^2));
pitch_meas_2d = atan2(ay,az);
z = [roll_meas; pitch_meas;pitch_meas_2d];

measRollPitch = z;
% if (roll_meas <= 0)
%     measRollPitch(1) = roll_meas+pi;
% else
%     measRollPitch(1) = roll_meas-pi;
% end
% 
% if (pitch_meas <= 0)
%     measRollPitch(2) = pitch_meas+pi;
% else
%     measRollPitch(2) = pitch_meas-pi;
% end

% persistent state
persistent x_est initialized
if isempty(initialized)
    x_est = single([measRollPitch(1); measRollPitch(2)]); % initialize to accel
    initialized = true;
end

% convert gyro to rad/s
gx = gx_deg * (pi/180);
gy = gy_deg * (pi/180);

% integrate gyro (note: we assume gx is roll rate, gy is pitch rate; confirm axes)
x_pred = x_est + dt * [gx; gy];

% complementary update
x_est = alpha * x_pred + (1 - alpha) * [measRollPitch(1); measRollPitch(2)];

tiltAngle = x_est;           % [roll; pitch] in radians
end


