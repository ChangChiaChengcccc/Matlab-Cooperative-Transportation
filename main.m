% A simulation for geometric tracking control of multirotors
close all;

% simulation time
dt = 0.001;
sim_t = 1;

%% initialize parameters
% iris1
iris1 = multirotor_dynamics;
iris1.m = 1.55;
iris1.J = [0.0347563, 0, 0;
                0, 0.0458929, 0;
                0, 0, 0.0977];
iris1.x =[0.6; 0; 0];
% iris2
iris2 = multirotor_dynamics;
iris2.m = 1.55;
iris2.J = [0.0347563, 0, 0;
                0, 0.0458929, 0;
                0, 0, 0.0977];
iris2.x =[-0.6; 0; 0];
% payload
payload = multirotor_dynamics;
payload.m = 0.3;
payload.J = [0.0031, 0, 0;
                0, 0.0656, 0;
                0, 0, 0.0656];
payload.x =[0; 0; 0.125];
system_inertia =  sys_inertia(iris1,iris2,payload);

% system
system = multirotor_dynamics;
system.dt = dt;
system.sim_t = sim_t;
system.t = 0:dt:sim_t;
system.m = system_inertia(1);
system.J = diag(system_inertia(2:4));
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

% initialize iris_dynamics
iris1_xva = zeros(9, length(system.t));
iris2_xva = zeros(9, length(system.t));

% initialize states
system.x(:, 1) = [0; 0; 0];
system.v(:, 1) = [0; 0; 0];
system.a(:, 1) = [0; 0; 0];
system.R(:, 1) = [1; 0; 0; 0; 1; 0; 0; 0; 1];
system.W(:, 1) = [0; 0; 0];
system.dW(:, 1) = [0; 0; 0];

% initialize controller
ctrl = controller;

% initialize trajectory
tra = zeros(12, length(system.t));
traj = trajectory;


%% control loop
for i = 2:length(system.t)
    t_now = system.t(i);

    % desired trajectory
    tra(:, i) = traj.traj_generate(t_now);
    Xd_enu = tra(1:9, i-1);
    b1d = tra(10:12, i);
    
    % control input and error
    [sys_fM, error] = ctrl.geometric_tracking_ctrl(i, system, Xd_enu, b1d);

    % system dynamics
    X0 = [vec_enu_to_ned(system.x(:, i-1));
        vec_enu_to_ned(system.v(:, i-1));
        reshape(reshape(system.R(:, i-1), 3, 3), 9, 1);
        system.W(:, i-1)];
    
    [T, X_new] = ode45(@(t, x) system.dynamics(t, x, sys_fM), [0, dt], X0, sys_fM);
    system.x(:, i) = vec_ned_to_enu(X_new(end, 1:3));
    system.v(:, i) = vec_ned_to_enu(X_new(end, 4:6));
    system.R(:, i) = X_new(end, 7:15);
    system.W(:, i) = X_new(end, 16:18);
    a_wdot = dvdW(system,i,sys_fM);
    system.a(:, i) = a_wdot(1:3);
    system.dW(:, i) = a_wdot(4:6);
    
    % Compute u*
    q_fM = ComputeUstar(iris1,iris2,system,sys_fM);
    
    % others dynamics
    iris1_xva(:, i) = iris_dynamics(iris1,system,i);
    iris2_xva(:, i) = iris_dynamics(iris2,system,i);
    
    % save the error_ned
    system.ex(:, i) = error(1:3);
    system.ev(:, i) = error(4:6);
    system.eR(:, i) = error(7:9);
    system.eW(:, i) = error(10:12);
    
    % save rotor thrust
    system.force_moment(:, i) = sys_fM(1:4);
    system.rotor_thrust(:, i) = system.allocation_matrix_inv*sys_fM(1:4);
end
%% plot 
% plot dynamics x
% iris1_to_iris2_x = [iris1_xva(1,:);iris2_xva(1,:)];
% iris1_to_iris2_y = [iris1_xva(2,:);iris2_xva(2,:)];
% iris1_to_iris2_z = [iris1_xva(3,:);iris2_xva(3,:)];
% plot3(iris1_to_iris2_x,iris1_to_iris2_y,iris1_to_iris2_z,'-k')
% hold on
% figure(8)
% plot3(system.x(1, :),system.x(2, :),system.x(3, :),...
%          iris1_xva(1,:),iris1_xva(2,:),iris1_xva(3,:), ...
%          iris2_xva(1,:),iris2_xva(2,:),iris2_xva(3,:));
% grid on

%check dynamics a
% plot3(system.x(1, 24000:30001),system.x(2, 24000:30001),system.x(3, 24000:30001))
% hold on
% grid on
% zlim([0 1.5])
% quiver3(system.x(1, 24000),system.x(2, 24000),system.x(3, 24000),system.a(1, 24000),system.a(2, 24000),system.a(3, 24000))
% 
% plot3(iris1_xva(1,24000:30001),iris1_xva(2,24000:30001),iris1_xva(3,24000:30001))
% quiver3(iris1_xva(1,24000),iris1_xva(2,24000),iris1_xva(3,24000),iris1_xva(7,24000),iris1_xva(8,24000),iris1_xva(9,24000))
% 
% quiver3(system.x(1, 25120),system.x(2, 25120),system.x(3, 25120),system.a(1, 25120),system.a(2, 25120),system.a(3, 25120))
% quiver3(iris1_xva(1,25120),iris1_xva(2,25120),iris1_xva(3,25120),iris1_xva(7,25120),iris1_xva(8,25120),iris1_xva(9,25120))
% 
% quiver3(system.x(1, 26690),system.x(2, 26690),system.x(3, 26690),system.a(1, 26690),system.a(2, 26690),system.a(3, 26690))
% quiver3(iris1_xva(1,26690),iris1_xva(2,26690),iris1_xva(3,26690),iris1_xva(7,26690),iris1_xva(8,26690),iris1_xva(9,26690))

% %check dynamics a
% plot3(system.x(1, 24000:30001),system.x(2, 24000:30001),system.x(3, 24000:30001))
% hold on
% grid on
% zlim([0 1.5])
% quiver3(system.x(1, 24000),system.x(2, 24000),system.x(3, 24000),system.v(1, 24000),system.v(2, 24000),system.v(3, 24000))
% 
% plot3(iris1_xva(1,24000:30001),iris1_xva(2,24000:30001),iris1_xva(3,24000:30001))
% quiver3(iris1_xva(1,24000),iris1_xva(2,24000),iris1_xva(3,24000),iris1_xva(4,24000),iris1_xva(5,24000),iris1_xva(6,24000))
% 
% quiver3(system.x(1, 25120),system.x(2, 25120),system.x(3, 25120),system.v(1, 25120),system.v(2, 25120),system.v(3, 25120))
% quiver3(iris1_xva(1,25120),iris1_xva(2,25120),iris1_xva(3,25120),iris1_xva(4,25120),iris1_xva(5,25120),iris1_xva(6,25120))
% 
% quiver3(system.x(1, 26690),system.x(2, 26690),system.x(3, 26690),system.v(1, 26690),system.v(2, 26690),system.v(3, 26690))
% quiver3(iris1_xva(1,26690),iris1_xva(2,26690),iris1_xva(3,26690),iris1_xva(4,26690),iris1_xva(5,26690),iris1_xva(6,26690))

% % plot trajectory and desired trajectory
% figure(1)
% subplot(3, 1, 1)
% plot(system.t, system.x(1, :))
% hold on
% plot(system.t, tra(1, :))
% y = ylabel('$X(m)$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% legend('$X$', '$X_{d}$' , 'Interpreter', 'latex')
% title('$Trajectory$ $and$ $Desired$ $Trajectory$ $(m)$', 'Interpreter', 'latex')
% subplot(3, 1, 2)
% plot(system.t, system.x(2, :))
% hold on
% plot(system.t, tra(2, :))
% y = ylabel('$Y(m)$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% legend('$Y$', '$Y_{d}$' , 'Interpreter', 'latex')
% subplot(3, 1, 3)
% plot(system.t, system.x(3, :))
% hold on
% plot(system.t, tra(3, :))
% y = ylabel('$Z(m)$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% legend('$Z$', '$Z_{d}$' , 'Interpreter', 'latex')
% xlabel('$Time(sec)$', 'Interpreter', 'latex')

% % plot position error
% figure(2)
% subplot(3, 1, 1)
% plot(system.t, system.ex(1, :))
% y = ylabel('$e_{p_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% title('$Position$ $Error$ $(m)$', 'Interpreter', 'latex')
% subplot(3, 1, 2)
% plot(system.t, system.ex(2, :))
% y = ylabel('$e_{p_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% subplot(3, 1, 3)
% plot(system.t, system.ex(3, :))
% y = ylabel('$e_{p_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% 
% % plot velocity error
% figure(3)
% subplot(3, 1, 1)
% plot(system.t, system.ev(1, :))
% y = ylabel('$e_{v_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% title('$Velocity$ $Error$ $(m/s)$', 'Interpreter', 'latex')
% subplot(3, 1, 2)
% plot(system.t, system.ev(2, :))
% y = ylabel('$e_{v_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% subplot(3, 1, 3)
% plot(system.t, system.ev(3, :))
% y = ylabel('$e_{v_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% 
% % plot attitude error
% figure(4)
% subplot(3, 1, 1)
% plot(system.t, system.eR(1, :))
% y = ylabel('$e_{R_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% title('$Attitude$ $Error$ $(rad)$', 'Interpreter', 'latex')
% subplot(3, 1, 2)
% plot(system.t, system.eR(2, :))
% y = ylabel('$e_{R_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% subplot(3, 1, 3)
% plot(system.t, system.eR(3, :))
% y = ylabel('$e_{R_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% 
% % plot angular velocity error
% figure(5)
% subplot(3, 1, 1)
% plot(system.t, system.eW(1, :))
% y = ylabel('$e_{\Omega_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% title('$Angular$ $Velocity$ $Error$ $(rad/s)$', 'Interpreter', 'latex')
% subplot(3, 1, 2)
% plot(system.t, system.eW(2, :))
% y = ylabel('$e_{\Omega_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% subplot(3, 1, 3)
% plot(system.t, system.eW(3, :))
% y = ylabel('$e_{\Omega_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% 
% figure(6)
% plot(system.t, system.rotor_thrust(1, :))
% hold on
% plot(system.t, system.rotor_thrust(2, :))
% hold on
% plot(system.t, system.rotor_thrust(3, :))
% hold on
% plot(system.t, system.rotor_thrust(4, :))
% y = ylabel('$f$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% legend('$f_{1}$', '$f_{2}$', '$f_{3}$', '$f_{4}$', 'Interpreter', 'latex')
% title('$Rotor$ $Thrust$ $(N)$', 'Interpreter', 'latex')
% 
% figure(7)
% subplot(211)
% plot(system.t, system.force_moment(1, :))
% y = ylabel('$f$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% title('$Total$ $thrust$ $(N)$', 'Interpreter', 'latex')
% 
% subplot(212)
% plot(system.t, system.force_moment(2, :))
% hold on
% plot(system.t, system.force_moment(3, :))
% hold on
% plot(system.t, system.force_moment(4, :))
% y = ylabel('$M$', 'rotation', 0, 'Interpreter', 'latex');
% set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
% xlabel('$Time(sec)$', 'Interpreter', 'latex')
% legend('$M_{x}$', '$M_{y}$', '$M_{z}$', 'Interpreter', 'latex')
% title('$Moment$ $Control$ $input$ $(N\cdot m)$', 'Interpreter', 'latex')
% figure(8)
% plot3()
