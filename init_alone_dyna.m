function object = init_alone_dyna(object,system)
    object.m = 1.55;
    object.J = [0.0347563, 0, 0;
                    0, 0.0458929, 0;
                    0, 0, 0.0977];
    object.x = zeros(3, length(system.t));
    object.v = zeros(3, length(system.t));
    object.a = zeros(3, length(system.t));
    object.R = zeros(9, length(system.t));
    object.W = zeros(3, length(system.t));
    object.dW = zeros(3, length(system.t));
    object.force_moment = zeros(4, length(system.t));
    object.rotor_thrust = zeros(4, length(system.t));
    object.fault_force_moment = zeros(4, length(system.t));
    object.fault_rotor_thrust = zeros(4, length(system.t));
    
    object.force = zeros(3, length(system.t));
    object.tau = zeros(3, length(system.t));
    object.d = 0.225;
    object.c_tau = 1.347e-2;
    object.allocation_matrix = cal_allocation_matrix(object.d, object.c_tau);
    object.allocation_matrix_inv = cal_allocation_matrix_inv(object.allocation_matrix);  
end