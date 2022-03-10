function ukf_state_estimation(iris1,iris1_alone)
    %% data 
    dt = iris1.dt;
    t_vec = iris1.t;
    % initial state
    initial_state = [iris1.x(:,1);iris1.v(:,1);iris1.a(:,1); iris1.W(:,1); iris1.dW(:,1); iris1_alone.E*0];
    true_state_vec = [iris1.x; iris1.v; iris1.a; iris1.W; iris1.dW; ...
                                iris1_alone.E(1)*ones(1,length(t_vec)); iris1_alone.E(2)*ones(1,length(t_vec)); ...
                                iris1_alone.E(3)*ones(1,length(t_vec)); iris1_alone.E(4)*ones(1,length(t_vec))];
    initial_measurement = [iris1.x(:,1); iris1.W(:,1); iris1_alone.force(:,1); iris1_alone.tau(:,1)];
    measurement_vec = [iris1.x; iris1.W; iris1_alone.force; iris1_alone.tau]; %+ 0.1*randn(length(initial_measurement),length(t_vec)); 
    
    %% ukf
    n=length(initial_state);      %number of state
    m=length(initial_measurement);     %number of measurement
    %std of process 
    q=[
          1e-3 1e-3 1e-3 ... %x 
          1e-3 1e-3 1e-3 ... %v 
          1e-1 1e-1 1e-1 ... %a
          1e-3 1e-3 1e-3 ... %W
          1e-1 1e-1 1e-1 ... %dW
          1e-3 1e-3 1e-3 1e-3 %E
         ];    
    %std of measurement
    r=[1e-3 1e-3 1e-3 ... %x
         1e-3 1e-3 1e-3 ... %W
         1e-3 1e-3 1e-3 ... %F
         1e-3 1e-3 1e-3 ... %tau
        ];    
    Q=diag(q)*diag(q);       % covariance of process
    R=diag(r)*diag(r);        % covariance of measurement  
    % nonlinear state equations
    f=@(x,iris1,iris1_alone,i) [% x
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
                                            x(9);
                                            % W
                                            x(10)+x(13)*dt;
                                            x(11)+x(14)*dt;
                                            x(12)+x(15)*dt;
                                            % dW
                                            x(13);
                                            x(14);
                                            x(15);
                                            % E
                                            x(16);
                                            x(17);
                                            x(18);
                                            x(19)
                                           ];
                                       
    % measurement equation
    h=@(x,iris1,iris1_alone,i) [%x
                                            x(1);
                                            x(2);
                                            x(3);
                                            %O
                                            x(10);
                                            x(11);
                                            x(12);
                                            %F
                                             iris1.m*[x(7);x(8);x(9)] ... 
                                            - vec_ned_to_enu(iris1.m*iris1.g*iris1.e3 - iris1_alone.forceE(x(16:19),iris1_alone,i)...
                                            *reshape(iris1.R(:,i),3,3)*iris1.e3);
                                            %tau
                                             iris1.J*[x(13);x(14);x(15)] ...
                                             - iris1_alone.momentE(x(16:19),iris1_alone,i) ...
                                             + cross([x(10); x(11); x(12)], iris1.J*[x(10); x(11); x(12)])
                                            ];                               
    x=initial_state;           
    P = eye(n);                                            % initial state covraiance
    N= length(t_vec);                                  % total dynamic steps
    xV = zeros(n,N);                                   % allocate memory
    for k=1:N
      z = measurement_vec(:,k);                % measurments
      [x, P] = ukf(f,x,P,h,z,Q,R,iris1,iris1_alone,k);            % ukf 
      xV(:,k) = x;                            % save estimate
    end

    %% plot
    %% xva
    figure(1)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(1,:),'k',t_vec, measurement_vec(1,:),'r',t_vec, xV(1,:),'b');
    title('2-D x Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$x\ ideal$','$x\ measurement$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(2,:),'k',t_vec, measurement_vec(2,:),'r',t_vec, xV(2,:),'b');
    title('2-D y Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$y\ ideal$','$y\ measurement$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,3)
    plot(t_vec, true_state_vec(3,:),'k',t_vec, measurement_vec(3,:),'r',t_vec, xV(3,:),'b');
    title('2-D z Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$z\ ideal$','$z\ measurement$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)

    figure(2)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(4,:),'k',t_vec, xV(4,:),'b');
    title('2-D vx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(5,:),'k',t_vec, xV(5,:),'b');
    title('2-D vy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(6,:),'k',t_vec, xV(6,:),'b');
    title('2-D vz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(3)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(7,:),'k',t_vec, xV(7,:),'b');
    title('2-D ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(8,:),'k',t_vec, xV(8,:),'b');
    title('2-D ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(9,:),'k',t_vec, xV(9,:),'b');
    title('2-D az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    %% W dW
    figure(5)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(10,:),'k',t_vec, xV(10,:),'b');
    title('2-D wx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(11,:),'k',t_vec, xV(11,:),'b');
    title('2-D wy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(12,:),'k',t_vec, xV(12,:),'b');
    title('2-D wz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(6)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(13,:),'k',t_vec, xV(13,:),'b');
    title('2-D dWx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(14,:),'k',t_vec, xV(14,:),'b');
    title('2-D dWy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(15,:),'k',t_vec, xV(15,:),'b');
    title('2-D dWz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    %% E
    figure(7)
    plot(t_vec, xV(16,:),'b',t_vec, xV(17,:),'c',t_vec, xV(18,:),'m',t_vec, xV(19,:),'y', ...
           t_vec, iris1_alone.E(1)*ones(1,length(t_vec)),'k', t_vec, iris1_alone.E(2)*ones(1,length(t_vec)),'k', ...
           t_vec, iris1_alone.E(3)*ones(1,length(t_vec)),'k', t_vec, iris1_alone.E(4)*ones(1,length(t_vec)),'k');
    ylim([0 2]);
    title('2-D E Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Efficiency$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$e1\ estimation$','$e2\ estimation$','$e3\ estimation$','$e4\ estimation$', 'Interpreter', 'latex','FontSize',15)
end
