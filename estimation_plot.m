function estimation_plot(t_vec,true_state_vec,measurement_vec,xV,iris1_alone,iris2_alone)
    %% iris1
    %xva
    figure(1)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(1,:),'k',t_vec, measurement_vec(1,:),'r',t_vec, xV(1,:),'b');
    title('2-D iris1 x Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$x\ ideal$','$x\ measurement$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(2,:),'k',t_vec, measurement_vec(2,:),'r',t_vec, xV(2,:),'b');
    title('2-D iris1 y Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$y\ ideal$','$y\ measurement$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,3)
    plot(t_vec, true_state_vec(3,:),'k',t_vec, measurement_vec(3,:),'r',t_vec, xV(3,:),'b');
    title('2-D iris1 z Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$z\ ideal$','$z\ measurement$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)

    figure(2)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(4,:),'k',t_vec, xV(4,:),'b');
    title('2-D iris1 vx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(5,:),'k',t_vec, xV(5,:),'b');
    title('2-D iris1 vy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(6,:),'k',t_vec, xV(6,:),'b');
    title('2-D iris1 vz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(3)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(7,:),'k',t_vec, xV(7,:),'b');
    title('2-D iris1 ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(8,:),'k',t_vec, xV(8,:),'b');
    title('2-D iris1 ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(9,:),'k',t_vec, xV(9,:),'b');
    title('2-D iris1 az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    % W dW
    figure(5)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(10,:),'k',t_vec, xV(10,:),'b');
    title('2-D iris1 wx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(11,:),'k',t_vec, xV(11,:),'b');
    title('2-D iris1 wy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(12,:),'k',t_vec, xV(12,:),'b');
    title('2-D iris1 wz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(6)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(13,:),'k',t_vec, xV(13,:),'b');
    title('2-D iris1 dWx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(14,:),'k',t_vec, xV(14,:),'b');
    title('2-D iris1 dWy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(15,:),'k',t_vec, xV(15,:),'b');
    title('2-D iris1 dWz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    % E
    figure(7)
    plot(t_vec, xV(16,:),'b',t_vec, xV(17,:),'c',t_vec, xV(18,:),'m',t_vec, xV(19,:),'y', ...
           t_vec, iris1_alone.E(1)*ones(1,length(t_vec)),'k', t_vec, iris1_alone.E(2)*ones(1,length(t_vec)),'k', ...
           t_vec, iris1_alone.E(3)*ones(1,length(t_vec)),'k', t_vec, iris1_alone.E(4)*ones(1,length(t_vec)),'k');
    ylim([0 2]);
    title('2-D iris1 E Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Efficiency$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$e1\ estimation$','$e2\ estimation$','$e3\ estimation$','$e4\ estimation$', 'Interpreter', 'latex','FontSize',15)
    
    %% iris2
    %xva
    figure(8)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(20,:),'k',t_vec, measurement_vec(13,:),'r',t_vec, xV(20,:),'b');
    title('2-D iris2 x Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$x(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$x\ ideal$','$x\ measurement$','$x\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(21,:),'k',t_vec, measurement_vec(14,:),'r',t_vec, xV(21,:),'b');
    title('2-D iris2 y Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$y(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$y\ ideal$','$y\ measurement$','$y\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,3)
    plot(t_vec, true_state_vec(22,:),'k',t_vec, measurement_vec(15,:),'r',t_vec, xV(22,:),'b');
    title('2-D iris2 z Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$z(m)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$z\ ideal$','$z\ measurement$','$z\ estimation$', 'Interpreter', 'latex','FontSize',20)

    figure(9)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(23,:),'k',t_vec, xV(23,:),'b');
    title('2-D iris2 vx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vx(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vx\ ideal$','$vx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(24,:),'k',t_vec, xV(24,:),'b');
    title('2-D iris2 vy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vy(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vy\ ideal$','$vy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(25,:),'k',t_vec, xV(25,:),'b');
    title('2-D iris2 vz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$vz(m/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$vz\ ideal$','$vz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(10)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(26,:),'k',t_vec, xV(26,:),'b');
    title('2-D iris2 ax Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ax(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ax\ ideal$','$ax\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(27,:),'k',t_vec, xV(27,:),'b');
    title('2-D iris2 ay Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$ay(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$ay\ ideal$','$ay\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(28,:),'k',t_vec, xV(28,:),'b');
    title('2-D iris2 az Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$az(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$az\ ideal$','$az\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    % W dW
    figure(12)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(29,:),'k',t_vec, xV(29,:),'b');
    title('2-D iris2 wx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wx(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wx\ ideal$','$wx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,2)
    plot(t_vec, true_state_vec(30,:),'k',t_vec, xV(30,:),'b');
    title('2-D iris2 wy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wy(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wy\ ideal$','$wy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(31,:),'k',t_vec, xV(31,:),'b');
    title('2-D iris2 wz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$wz(rad/s)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$wz\ ideal$','$wz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    figure(13)
    subplot(3,1,1)
    plot(t_vec, true_state_vec(32,:),'k',t_vec, xV(32,:),'b');
    title('2-D iris2 dWx Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWx(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWx\ ideal$','$dWx\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    subplot(3,1,2)
    plot(t_vec, true_state_vec(33,:),'k',t_vec, xV(33,:),'b');
    title('2-D iris2 dWy Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWy(rad/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWy\ ideal$','$dWy\ estimation$', 'Interpreter', 'latex','FontSize',20)
    subplot(3,1,3)
    plot(t_vec, true_state_vec(34,:),'k',t_vec, xV(34,:),'b');
    title('2-D iris2 dWz Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$dWz(m/s^2)$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$dWz\ ideal$','$dWz\ estimation$', 'Interpreter', 'latex','FontSize',20)
    
    % E
    figure(14)
    plot(t_vec, xV(35,:),'b',t_vec, xV(36,:),'c',t_vec, xV(37,:),'m',t_vec, xV(38,:),'y', ...
           t_vec, iris2_alone.E(1)*ones(1,length(t_vec)),'k', t_vec, iris2_alone.E(2)*ones(1,length(t_vec)),'k', ...
           t_vec, iris2_alone.E(3)*ones(1,length(t_vec)),'k', t_vec, iris2_alone.E(4)*ones(1,length(t_vec)),'k');
    ylim([0 2]);
    title('2-D iris2 E Plot','FontSize',20);
    x = xlabel('$t$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    y = ylabel('$Efficiency$', 'rotation', 0, 'Interpreter', 'latex','FontSize',20);
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    legend('$e1\ estimation$','$e2\ estimation$','$e3\ estimation$','$e4\ estimation$', 'Interpreter', 'latex','FontSize',15)
end