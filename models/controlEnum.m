classdef controlEnum < Simulink.IntEnumType

    enumeration
        LQR(0)
        LQRmotor(1)
        MPC_control(2)
        PID(3)
    end

end
