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