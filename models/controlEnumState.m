classdef controlEnumState < Simulink.IntEnumType

    enumeration
        LQR(0)
        LQRmotor(1)
        MPC_control(2)
        PID_State(3)
    end

end
