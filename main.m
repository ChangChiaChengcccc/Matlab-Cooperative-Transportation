% A simulation for geometric tracking control of multirotors
close all;
%% initialize
% simulation time
dt = 0.001;
sim_t = 20;

% initialize iris1,iris2,payload,system
iris1 = multirotor_dynamics;
iris2 = multirotor_dynamics;
payload = multirotor_dynamics;
system = multirotor_dynamics;
[iris1, iris2, payload, system] = init_sys(iris1,iris2,payload,system,dt,sim_t);

% initialize iris1_alone, iris2_alone
iris1_alone = multirotor_dynamics;
iris2_alone = multirotor_dynamics;
iris1_alone = init_alone_dyna(iris1_alone,system);
iris2_alone = init_alone_dyna(iris2_alone,system);

% initialize controller
ctrl = controller;

% initialize trajectory
tra = zeros(12, length(system.t));
traj = trajectory;

% test
test_data =  zeros(3, length(system.t)); %W from dW

%% control loop
for i = 2:length(system.t)
    t_now = system.t(i);

    % desired trajectory
    tra(:, i) = traj.traj_generate(t_now);
    Xd_enu = tra(1:9, i-1);
    b1d = tra(10:12, i);
    
    % control input and error
    [sys_fM, sys_error] = ctrl.geometric_tracking_ctrl(i, system, Xd_enu, b1d);

    % system dynamics
    sys_X0 = [vec_enu_to_ned(system.x(:, i-1));
        vec_enu_to_ned(system.v(:, i-1));
        reshape(reshape(system.R(:, i-1), 3, 3), 9, 1);
        system.W(:, i-1)];
    
    [T, X_new] = ode45(@(t, x) system.dynamics(t, x, sys_fM), [0, dt], sys_X0, sys_fM);
    system.x(:, i) = vec_ned_to_enu(X_new(end, 1:3));
    system.v(:, i) = vec_ned_to_enu(X_new(end, 4:6));
    system.R(:, i) = X_new(end, 7:15);
%     ypr= rotm2eul(reshape(system.R(:,i),3,3),'ZYX');
%     system.rpy(:,i) =[ypr(3);ypr(2);ypr(1)];
%     system.rpy(:,i) = rotmat2eulang(reshape(system.R(:,i),3,3),'?');
%     quat = rotm2quat(reshape(system.R(:,i),3,3));
%     quat = quaternion(quat);
%     eulerAnglesDegrees = eulerd(quat,'XZX','frame');
    system.rpy(:,i) = system.rpy(:,i-1)+system.W(:, i-1)*dt;
    system.W(:, i) = X_new(end, 16:18);
    [system.a(:, i), system.dW(:, i)] = dvdW(system,i,sys_fM);
    
    % test
    test_data(:,i) = test_data(:,i-1) + (system.dW(:, i)+system.dW(:, i-1))*0.5*dt;
    
    % save the error_ned
    system.ex(:, i) = sys_error(1:3);
    system.ev(:, i) = sys_error(4:6);
    system.eR(:, i) = sys_error(7:9);
    system.eW(:, i) = sys_error(10:12);
    
    % save rotor thrust
    system.force_moment(:, i) = sys_fM;
    system.rotor_thrust(:, i) = system.allocation_matrix_inv*sys_fM;
    
    % others dynamics (with system)
    [iris1.x(:, i), iris1.v(:, i), iris1.a(:, i)] = iris_dynamics(iris1,system,i);
    [iris2.x(:, i), iris2.v(:, i), iris2.a(:, i)] = iris_dynamics(iris2,system,i);
    iris1.R(:,i) =  system.R(:, i);
    iris1.rpy(:,i) = system.rpy(:,i);
    iris1.W(:,i) = system.W(:, i);
    iris1.dW(:,i)  = system.dW(:, i);
    iris2.R(:,i) =  system.R(:, i);
    iris2.rpy(:,i) = system.rpy(:,i);
    iris2.W(:,i) = system.W(:, i);
    iris2.dW(:,i)  = system.dW(:, i);
        
   %% get F & tau (alone)
    % Compute u*
    [iris1_alone.force_moment(:,i), iris2_alone.force_moment(:,i)]= ComputeUstar(iris1,iris2,system,sys_fM);
    
    % iris1_alone dynamics
    iris1_X0 = [vec_enu_to_ned(iris1.x(:, i-1));
        vec_enu_to_ned(iris1.v(:, i-1));
        % The orientation is the same
        reshape(reshape(system.R(:, i-1), 3, 3), 9, 1);
        system.W(:, i-1)];
    [T, X_new] = ode45(@(t, x) iris1.dynamics(t, x, iris1_alone.force_moment(:,i)), [0, dt], iris1_X0, iris1_alone.force_moment(:,i));
    iris1_alone.x(:, i) = vec_ned_to_enu(X_new(end, 1:3));
    iris1_alone.v(:, i) = vec_ned_to_enu(X_new(end, 4:6));
    iris1_alone.R(:, i) = X_new(end, 7:15);
    iris1_alone.W(:, i) = X_new(end, 16:18);
    [iris1_alone.a(:, i), iris1_alone.dW(:, i)] = dvdW(iris1_alone,i,iris1_alone.force_moment(:,i));
    % F and tau on iris1
    [iris1_alone.force(:, i), iris1_alone.tau(:, i)] = force_tau(iris1,iris1_alone,i);
    % efficiency
    iris1_alone.rotor_thrust(:, i) = iris1_alone.allocation_matrix_inv*iris1_alone.force_moment(:,i);
    iris1_alone.fault_force_moment = iris1_alone.allocation_matrix*diag(iris1_alone.E)*iris1_alone.rotor_thrust(:, i);
    
    
    % iris2_alone dynamics
    iris2_X0 = [vec_enu_to_ned(iris2.x(:, i-1));
        vec_enu_to_ned(iris2.v(:, i-1));
        % The orientation is the same
        reshape(reshape(system.R(:, i-1), 3, 3), 9, 1);
        system.W(:, i-1)];
    [T, X_new] = ode45(@(t, x) iris2.dynamics(t, x, iris2_alone.force_moment(:,i)), [0, dt], iris2_X0, iris2_alone.force_moment(:,i));
    iris2_alone.x(:, i) = vec_ned_to_enu(X_new(end, 1:3));
    iris2_alone.v(:, i) = vec_ned_to_enu(X_new(end, 4:6));
    iris2_alone.R(:, i) = X_new(end, 7:15);
    iris2_alone.W(:, i) = X_new(end, 16:18);
    [iris2_alone.a(:, i), iris2_alone.dW(:, i)] = dvdW(iris2_alone,i,iris2_alone.force_moment(:,i));
    % F and tau on iris2
    [iris2_alone.force(:, i), iris2_alone.tau(:, i)] = force_tau(iris2,iris2_alone,i);
    % efficiency
    iris2_alone.rotor_thrust(:, i) = iris2_alone.allocation_matrix_inv*iris2_alone.force_moment(:,i);
    iris2_alone.fault_force_moment = iris2_alone.allocation_matrix*diag(iris2_alone.E)*iris2_alone.rotor_thrust(:, i);
end
%% check fM
% result = test_fM(system,iris1,iris2,iris1_alone,iris2_alone)
%% ukf
ukf_state_estimation(iris1,iris1_alone);
%% chiacheng plot 
%plot_function(iris1,iris2,system) %sim_t > 30

% compare W and W integral from dW
% subplot(3,1,1)
% plot(system.t, system.W(1, :),'k',system.t, test_data(1,:),'b');
%     title('2-D Compare Wx Plot','FontSize',20);
%     x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     y = ylabel('$Wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
%     legend('$Wx\ ideal$','$Wx\ integral$', 'Interpreter', 'latex','FontSize',20)
% subplot(3,1,2)
% plot(system.t, system.W(2, :),'k',system.t, test_data(2,:),'b');
%     title('2-D Compare Wy Plot','FontSize',20);
%     x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     y = ylabel('$Wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
%     legend('$Wy\ ideal$','$Wy\ integral$', 'Interpreter', 'latex','FontSize',20)
% subplot(3,1,3)
% plot(system.t, system.W(3, :),'k',system.t, test_data(3,:),'b');
%     title('2-D Compare Wx Plot','FontSize',20);
%     x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     y = ylabel('$Wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
%     set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
%     legend('$Wz\ ideal$','$Wz\ integral$', 'Interpreter', 'latex','FontSize',20)


%% chengcheng original plot
% plot trajectory and desired trajectory
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
% 
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

