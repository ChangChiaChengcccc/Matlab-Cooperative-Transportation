function ukf_state_estimation(iris1,iris1_alone)
    %% data 
    dt = iris1.dt;
    t_vec = iris1.t;
    % initial state
    initial_state = [iris1.x(:,1);iris1.v(:,1);iris1.a(:,1)];
    state_vec = [iris1.x;iris1.v;iris1.a];
    initial_measurement = iris1.x(:,1);
    measurement_vec = iris1.x + randn(length(initial_measurement),length(t_vec)); 

    %% ukf
    n=length(initial_state);      %number of state
    m=length(initial_measurement);     %number of measurement
    q=[1e-4 1e-4 1e-4 1e-1 1e-1 1e-1 0.3 0.3 0.3];    %std of process 
    r=[1 1 1];    %std of measurement
    Q=diag(q)*diag(q); % covariance of process
    R=diag(r)*diag(r);        % covariance of measurement  
    % nonlinear state equations
    f=@(x) [  % x
                    x(1)+x(4)*dt+0.5*x(7)*dt^2;
                    x(2)+x(5)*dt+0.5*x(8)*dt^2;
                    x(3)+x(6)*dt+0.5*x(9)*dt^2;   
                    % v
                    x(4)+x(7)*dt;
                    x(5)+x(8)*dt;
                    x(6)+x(9)*dt;
                    % a
                    x(7);
                    x(8);
                    x(9)
                 ];
    % measurement equation
    h=@(x) [  % x
                    x(1);
                    x(2);
                    x(3)
                 ];                               
    x=initial_state;           
    P = eye(n);                                            % initial state covraiance
    N= length(t_vec);                                  % total dynamic steps
    xV = zeros(n,N);                                   % allocate memory
    for k=1:N
      z = measurement_vec(:,k);                % measurments
      [x, P] = ukf(f,x,P,h,z,Q,R);            % ukf 
      xV(:,k) = x;                            % save estimate
    end

    %% error 
%     ori_err_vec = state_vec(1,:) - state_vec_noise(1,:);
%     ori_err = rms(ori_err_vec)
%     fil_err_vec = state_vec(1,:) - xV(1,:);
%     fil_err = rms(fil_err_vec)

    %% plot
    figure(1)
    subplot(3,1,1)
    plot(t_vec, state_vec(1,:),'k',t_vec, measurement_vec(1,:),'r',t_vec, xV(1,:),'b');
    title('2-D x Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$x\ ideal$','$x\ measurement$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, state_vec(2,:),'k',t_vec, measurement_vec(2,:),'r',t_vec, xV(2,:),'b');
    title('2-D y Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$y\ ideal$','$y\ measurement$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,3)
    plot(t_vec, state_vec(3,:),'k',t_vec, measurement_vec(3,:),'r',t_vec, xV(3,:),'b');
    title('2-D z Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$z\ ideal$','$z\ measurement$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)

    figure(2)
    subplot(3,1,1)
    plot(t_vec, state_vec(4,:),'k',t_vec, xV(4,:),'b');
    title('2-D vx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, state_vec(5,:),'k',t_vec, xV(5,:),'b');
    title('2-D vy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, state_vec(6,:),'k',t_vec, xV(6,:),'b');
    title('2-D vz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(3)
    subplot(3,1,1)
    plot(t_vec, state_vec(7,:),'k',t_vec, xV(7,:),'b');
    title('2-D ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, state_vec(8,:),'k',t_vec, xV(8,:),'b');
    title('2-D ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, state_vec(9,:),'k',t_vec, xV(9,:),'b');
    title('2-D az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
end
