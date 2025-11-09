motorParamsDD = Simulink.data.dictionary.open('motorParametersData.sldd');
robotParamsDD = Simulink.data.dictionary.open('robotModelData.sldd');
controlParamsDD = Simulink.data.dictionary.open('controlVariablesData.sldd');
modelParamsDD = Simulink.data.dictionary.open('modelConfigData.sldd');
sectMotor = getSection(motorParamsDD,'Design Data');
sectRobot = getSection(robotParamsDD,'Design Data');
sectControl = getSection(controlParamsDD,'Design Data');
sectModel = getSection(modelParamsDD,'Design Data');

%% Physical Parameters
M = getValue(getEntry(sectRobot,'massOfCart'));    % mass of cart
m = getValue(getEntry(sectRobot,'massOfPendulum'));    % mass of pendulum
L = getValue(getEntry(sectRobot,'lengthToPendulumCenterOfMass'));    % length to pendulum center of mass
g = 9.81;   % gravity
I = getValue(getEntry(sectRobot,'inertiaOfPendulum'));  % inertia of pendulum
r = getValue(getEntry(sectRobot,'wheelRadius'));   % 5 cm wheel radius

% Servo motor state: i, omega
Rm = getValue(getEntry(sectMotor,'armatureResistance')); % motor resistance
Lm = getValue(getEntry(sectMotor,'armatureInductance')); % motor coil inductance
Km = getValue(getEntry(sectMotor,'motorTorqueConstantKm')); % motor torque constant
Kb = getValue(getEntry(sectMotor,'backEMFConstantKb')); % motor electrical consant
Kf = getValue(getEntry(sectMotor,'viscousFriction')); % viscous friction
J = getValue(getEntry(sectMotor,'motorInertia'));  % inertia of rotor

dt = getValue(getEntry(sectModel,'controlRate'));

N_gear = 42;

% States are x, dx/dt, phi, dphi/dt
% Outputs are x position and tilt angle

% Linearized state-space model of inverted pendulum
I_tot = m*L^2+I;
p = m^2*L^2 - I_tot*(M+m);
A_pend = [0 1 0 0;
          0 0 (m^2 * g * L^2)/p 0;
          0 0 0 1;
          0 0 -(M+m)/(m*L)*m^2*L^2*g/p 0];
      
B_pend = [0; -I_tot/p; 0; 1/(m*L)*(1+(M+m)*I_tot/p)];

C_pend = [1 0 0 0;
          0 0 1 0];
D_pend = zeros(2,1);


% State-space model: output is torque = Km * i
A_motor = [-Rm/L,   -Kb/Lm;
            Km/J, -Kf/J];
B_motor = [1/Lm;
           0];
C_motor = [(N_gear*Km*(1/r)), 0];  % Output is torque = Km * i
D_motor = 0;

% Augment System: Force = torque / r
% States are x, dx/dt, phi, dphi/dt, i, w
% Outputs are x position and tilt angle

A_aug = [A_pend, B_pend*C_motor;
         zeros(2,4), A_motor];

B_aug = [B_pend*D_motor; B_motor];

C_aug = [C_pend, D_pend*C_motor]; 
n_aug = size(A_aug,1);

% Discretize System - for the augmented system
sysd = c2d(ss(A_aug, B_aug, C_aug, 0), dt);
Ad_aug = sysd.A;
Bd_aug = sysd.B;
Cd_aug = sysd.C;

% LQR Controller - for the augmented system
% Q = diag([1/0.02^2 1/0.1^2 1/(10*pi/180)^2 1/0.1^2 1 1]);
Q = diag([80,1,80,1,0.1,0.0001]);
R_lqr = 30;                        
[K_lqr, ~, ~] = dlqr(Ad_aug, Bd_aug, Q, R_lqr);

% Kalman Filter - for the augmented system
Q_kf = diag([0.2 0.2 0.2 0.2 0.2 0.2]);   % Slightly higher process noise
R_kf = diag([0.001 0.001]);                 % Lower measurement noise (trust sensors more)
