load MPC1Data.mat % horizon = 200, controlHor = 10, 100 Hz MPC, Weights.OV = [1 0 10 0 0 0], Weights.MVRate = 0.1, constrainst same as physical
                  % C = eye(6) used for 6 measured outputs from Kalman
logs = out.logsout;
y1 = logs{1}.Values.Data;
y2 = logs{2}.Values.Data;
x_hat_states = logs{3}.Values.Data;
u = logs{4}.Values.Data;
t = logs{1}.Values.Time;

figure()
subplot(2,1,1)
plot(t,y1)
xlabel('t [s]')
grid on
subplot(2,1,2)
plot(t,y2)
xlabel('t [s]')
grid on
title('Output')

figure()
subplot(6,1,1)
plot(t,x_hat_states(:,1))
xlabel('t [s]')
grid on
subplot(6,1,2)
plot(t,x_hat_states(:,2))
xlabel('t [s]')
grid on

subplot(6,1,3)
plot(t,x_hat_states(:,3))
xlabel('t [s]')
grid on
subplot(6,1,4)
plot(t,x_hat_states(:,5))
xlabel('t [s]')
grid on

subplot(6,1,5)
plot(t,x_hat_states(:,5))
xlabel('t [s]')
grid on
subplot(6,1,6)
plot(t,x_hat_states(:,6))
xlabel('t [s]')
grid on
title('State Vector')

figure()
plot(t,u)
xlabel('t [s]')
grid on
title('Input')
