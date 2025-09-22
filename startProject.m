% Clear workspace
clear

% Clear all requirements and requirement links loaded in memory
slreq.clear

% Robot and motor parameter initialization
J = 0.01;   % motor inertia
Kt = 0.01;  % motor torque constant
M = 0.5;    % mass of cart
m = 0.2;    % mass of pendulum
l = 0.3;    % length to pendulum center of mass
g = 9.81;   % gravity
I = 0.006;  % inertia of pendulum
r = 0.05;   % 5 cm wheel radius

% States are x, dx/dt, phi, dphi/dt
% Outputs are x position and tilt angle

% Linearized state-space model of inverted pendulum
p = m*l^2+I-(m^2*l^2)/(M+m);
A_pend = [0 1 0 0;
          0 0 -(m^2 * g * l^2)/(p*(m+M)) 0;
          0 0 0 1;
          0 0 (m * g * l)/p 0];
      
B_pend = [0; (m^2*l^2*g)/p*1/(m+M); 0; -(m*l)/p];

C_pend = [1 0 0 0;
          0 0 1 0];
D_pend = zeros(2,1);

% LQR Controller - for the augmented system
Q_lqr = diag([1/0.02^2 1/0.1^2 1/(10*pi/180)^2 1/0.1^2]);
R_lqr = 1;                        
[K_lqr, ~, ~] = dlqr(A_pend, B_pend, Q_lqr, R_lqr);

% Servo motor state: i, omega
R = 2.0;
L = 0.5;
Km = 0.1; 
Kb = 0.1; 
Kf = 0.2;
J = 0.02;

% Transfer function can be created with torque output

% H1 = tf(Km,[L R]);    
% H2 = tf(1,[J Kf]);  
% dcm=feedback(H1,H2*Kb);
% [A_motor,B_motor,C_motor,D_motor]= tf2ss(dcm.Numerator{1},dcm.Denominator{1})

% State-space model: output is torque = Km * i
A_motor = [-R/L,   -Kb/L;
            Km/J, -Kf/J];
B_motor = [1/L;
           0];
C_motor = [Km, 0];  % Output is torque = Km * i
D_motor = 0;

% Augment System: Force = torque / r
% States are x, dx/dt, phi, dphi/dt, i, w
% Outputs are x position and tilt angle

A_aug = [A_pend, (1/r)*B_pend*C_motor;
         zeros(2,4), A_motor];

B_aug = [(1/r)*B_pend*D_motor; B_motor];

C_aug = [C_pend, (1/r)*D_pend*C_motor]; 
n_aug = size(A_aug,1);

% Discretize System - for the augmented system
dt = 0.01;
sysd = c2d(ss(A_aug, B_aug, C_aug, 0), dt);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

% LQR Controller - for the augmented system
Q_aug = diag([1/0.02^2 1/0.1^2 1/(10*pi/180)^2 1/0.1^2 1 1]);
R_aug_lqr = 1;                        
[K_aug_lqr, ~, ~] = dlqr(Ad, Bd, Q_aug, R_aug_lqr);

% Kalman Filter - for the augmented system
Q_kf = diag([0.02 0.02 0.02 0.02]);   % Slightly higher process noise
Q_kf_aug = diag([0.02 0.02 0.02 0.02 0.02 0.02]);   % Slightly higher process noise
R_kf = 0.01*diag([0.001 0.001]);                 % Lower measurement noise (trust sensors more)