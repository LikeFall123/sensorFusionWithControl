xm = 0.01;
xdm = 2;
pm = 0.02;
pdm = 5;
im = 2;
wm = 100;

Q_aug_lqr_temp = 0.1*diag([(1/xm)^2,(1/xdm)^2,(1/pm)^2,(1/pdm)^2,(1/im)^2,(1/wm)^2]);
R_aug_lqr_temp = 15;

% LQR Controller - for the augmented system                     
[K, ~, ~] = dlqr(Ad_aug, Bd_aug, Q_aug_lqr_temp, R_aug_lqr_temp);
K = single(K)

% Velocity angle PID
Tw_vel = 1;
Kp_vel = 0.01;
Ki_vel = 0.01; 
Kd_vel = 0;

% Position angle PID
targetPosition = 0.1;
Tw_pos = 1;
Kp_pos = 0.1;
Ki_pos = 0.0001; 
Kd_pos = 0.1;