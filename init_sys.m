function [iris1, iris2, payload, system] = init_sys(iris1,iris2,payload,system,dt,sim_t)
   %% iris1, iris2, payload
    % initialize parameters
    iris1.dt = dt;
    iris1.sim_t = sim_t;
    iris1.t = 0:dt:sim_t;
    iris1.x =zeros(3, length(iris1.t));
    iris1.v =zeros(3, length(iris1.t));
    iris1.a =zeros(3, length(iris1.t));
    iris1.R = zeros(9, length(iris1.t));
    iris1.W = zeros(3, length(iris1.t));
    iris1.dW =zeros(3, length(iris1.t));
    iris1.d = 0.225;
    iris1.c_tau = 1.347e-2;
    iris1.allocation_matrix = cal_allocation_matrix(iris1.d, iris1.c_tau);
    iris1.allocation_matrix_inv = cal_allocation_matrix_inv(iris1.allocation_matrix);  
    
    iris2.dt = dt;
    iris2.sim_t = sim_t;
    iris2.t = 0:dt:sim_t;
    iris2.x =zeros(3, length(iris2.t));
    iris2.v =zeros(3, length(iris2.t));
    iris2.a =zeros(3, length(iris2.t));
    iris2.R = zeros(9, length(iris2.t));
    iris2.W = zeros(3, length(iris2.t));
    iris2.dW =zeros(3, length(iris2.t));  
    iris2.d = 0.225;
    iris2.c_tau = 1.347e-2;
    iris2.allocation_matrix = cal_allocation_matrix(iris2.d, iris2.c_tau);
    iris2.allocation_matrix_inv = cal_allocation_matrix_inv(iris2.allocation_matrix);  
    
    iris1.m = 1.55;
    iris1.J = [0.0347563, 0, 0;
                    0, 0.0458929, 0;
                    0, 0, 0.0977];
    iris1.x(:,1) =[0.6; 0; 0];
    
    iris2.m = 1.55;
    iris2.J = [0.0347563, 0, 0;
                    0, 0.0458929, 0;
                    0, 0, 0.0977];
    iris2.x(:,1) =[-0.6; 0; 0];
    
    payload.m = 0.3;
    payload.J = [0.0031, 0, 0;
                    0, 0.0656, 0;
                    0, 0, 0.0656];
    payload.x =[0; 0; 0];

   %% system
    % initialize parameters
    system =  sys_inertia(iris1,iris2,payload,system);
    system.dt = dt;
    system.sim_t = sim_t;
    system.t = 0:dt:sim_t;
    system.d = 0.225;
    system.c_tau = 1.347e-2;
    system.allocation_matrix = cal_allocation_matrix(system.d, system.c_tau);
    system.allocation_matrix_inv = cal_allocation_matrix_inv(system.allocation_matrix);
    system.x = zeros(3, length(system.t));
    system.v = zeros(3, length(system.t));
    system.a = zeros(3, length(system.t));
    system.R = zeros(9, length(system.t));
    system.W = zeros(3, length(system.t));
    system.dW = zeros(3, length(system.t));
    system.ex = zeros(3, length(system.t));
    system.ev = zeros(3, length(system.t));
    system.eR = zeros(3, length(system.t));
    system.eW = zeros(3, length(system.t));
    system.force_moment = zeros(4, length(system.t));
    system.rotor_thrust = zeros(4, length(system.t));
    system.fault_force_moment = zeros(4, length(system.t));
    system.fault_rotor_thrust = zeros(4, length(system.t));
    
    % initialize states
    system.x(:, 1) = [0; 0; 0];
    system.v(:, 1) = [0; 0; 0];
    system.a(:, 1) = [0; 0; 0];
    system.R(:, 1) = [1; 0; 0; 0; 1; 0; 0; 0; 1];
    system.W(:, 1) = [0; 0; 0];
    system.dW(:, 1) = [0; 0; 0];
end