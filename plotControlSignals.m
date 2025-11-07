% load PID1.mat
% Lehet még több PID-es (de működik)

load PIDState.mat
% PIDState: Paramok doksiban leírva
%   - 160 - 190 mp, vagy 200 -> mp a hasznos jel

% load LQR.mat
% LQR control: Q = 0.1*(1/x_i_max)^2
%              R = 15
%              Kalman params: ugyanaz mint PIDState-nél
%              198-220 s felvétel

% scenario = "PID";
% scenario = "PIDState";
scenario = "LQR";
plotStateVector = 1;

if scenario == "PID"
    compFilt = find(logsout,'compFilt').Values.Data;
    controlVoltage = find(logsout,'motor_control_voltage').Values.Data;
    t = find(logsout,'compFilt').Values.Time;
    
    figure()
    subplot(2,1,1)
    plot(t,compFilt(:,2))
    xlabel('t [s]')
    ylabel('tiltAngle [rad]')
    grid on
    
    subplot(2,1,2)
    plot(t,controlVoltage)
    xlabel('t [s]')
    ylabel('Voltage [V]')
    grid on
else
    if scenario == "PIDState" || scenario == "LQR"
        compFilt = find(logsout,'compFilt').Values.Data;
        controlVoltage = find(logsout,'motor_control_voltage').Values.Data;
        stateVector = find(logsout,'state_vector').Values.Data;
        t = find(logsout,'compFilt').Values.Time;
        t_start = 0;
        t_end = t(end);
        
        figure()
        subplot(2,1,1)
        plot(t,compFilt(:,2))
        hold on
        plot(t,stateVector(:,3))
        hold off
        xlabel('t [s]')
        ylabel('tiltAngle [rad]')
        legend('Complementary filer','Estimated state')
        grid on
        xlim([t_start t_end])
        
        subplot(2,1,2)
        plot(t,controlVoltage)
        xlabel('t [s]')
        ylabel('Voltage [V]')
        grid on
        xlim([t_start t_end])
    end

    if plotStateVector == 1
        figure()
        subplot(6,1,1)
        plot(t,stateVector(:,1))
        xlabel('t [s]')
        ylabel('Position [m]')
        grid on
        xlim([t_start t_end])

        subplot(6,1,2)
        plot(t,stateVector(:,2))
        xlabel('t [s]')
        ylabel('Velocity [m/s]')
        grid on
        xlim([t_start t_end])

        subplot(6,1,3)
        plot(t,stateVector(:,3))
        xlabel('t [s]')
        ylabel('Pitch [rad]')
        grid on
        xlim([t_start t_end])

        subplot(6,1,4)
        plot(t,stateVector(:,4))
        xlabel('t [s]')
        ylabel('Angular velocity [rad/s]')
        grid on
        xlim([t_start t_end])

        subplot(6,1,5)
        plot(t,stateVector(:,5))
        xlabel('t [s]')
        ylabel('Current [A]')
        grid on
        xlim([t_start t_end])

        subplot(6,1,6)
        plot(t,stateVector(:,6))
        xlabel('t [s]')
        ylabel('Motor speed [m]')
        grid on
        xlim([t_start t_end])
        
    end


end
