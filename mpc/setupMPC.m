dt = 1/100;
sysd = c2d(ss(A_aug, B_aug, eye(6), zeros(6,1)), dt);
Ad_aug = sysd.A;
Bd_aug = sysd.B;
Cd_aug = sysd.C;

%% create MPC controller object with sample time
mpc1 = mpc(sysd, dt);
%% specify prediction horizon
mpc1.PredictionHorizon = 100;
%% specify control horizon
mpc1.ControlHorizon = 5;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = [0;0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -12;
mpc1.MV(1).Max = 12;
%% specify constraints for OV
mpc1.OV(1).Min = -0.1;
mpc1.OV(1).Max = 0.1;
mpc1.OV(2).Min = -2;
mpc1.OV(2).Max = 2;
mpc1.OV(3).Min = -0.01;
mpc1.OV(3).Max = 0.01;
mpc1.OV(4).Min = -2;
mpc1.OV(4).Max = 2;
mpc1.OV(5).Min = -2;
mpc1.OV(5).Max = 2;
mpc1.OV(6).Min = -100;
mpc1.OV(6).Max = 100;
%% specify overall adjustment factor applied to weights
beta = 6.821;
%% specify weights
mpc1.Weights.MV = 0*beta;
mpc1.Weights.MVRate = 0.1/beta;
mpc1.Weights.OV = [1 0 10 0 0 0];%*beta;
mpc1.Weights.ECR = 100000;
%% specify overall adjustment factor applied to estimation model gains
alpha = 4.1687;
%% adjust default output disturbance model gains
setoutdist(mpc1, 'model', getoutdist(mpc1)*alpha);
%% adjust default measurement noise model gains
mpc1.Model.Noise = mpc1.Model.Noise/alpha;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 10001, mpc1_RefSignal, mpc1_MDSignal, options);
