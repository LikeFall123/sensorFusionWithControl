classdef mainStateEnum < Simulink.IntEnumType

    enumeration
        Main_Kalman_1D(0)
        Main_Kalman_1D_Augmented(1)
        Main_Kalman_1D_Augmented_Block(2)
        Main_Kalman_2D(3)
        Main_Kalman_2D_Augmented(4)
    end

end
