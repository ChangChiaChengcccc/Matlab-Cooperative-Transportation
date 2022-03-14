function ukf_state_estimation(iris1,iris1_alone,iris2,iris2_alone)
    %% data 
    dt = iris1.dt;
    t_vec = iris1.t;
    % initial state
    initial_state = [
                            iris1.x(:,1);iris1.v(:,1);iris1.a(:,1); iris1.W(:,1); iris1.dW(:,1); iris1_alone.E*0; ...
                            iris2.x(:,1);iris2.v(:,1);iris2.a(:,1); iris2.W(:,1); iris2.dW(:,1); iris2_alone.E*0
                         ];
    true_state_vec = [
                                    iris1.x; iris1.v; iris1.a; iris1.W; iris1.dW; ...
                                    iris1_alone.E(1)*ones(1,length(t_vec)); iris1_alone.E(2)*ones(1,length(t_vec)); ...
                                    iris1_alone.E(3)*ones(1,length(t_vec)); iris1_alone.E(4)*ones(1,length(t_vec)); ...
                                    iris2.x; iris2.v; iris2.a; iris2.W; iris2.dW; ...
                                    iris2_alone.E(1)*ones(1,length(t_vec)); iris2_alone.E(2)*ones(1,length(t_vec)); ...
                                    iris2_alone.E(3)*ones(1,length(t_vec)); iris2_alone.E(4)*ones(1,length(t_vec))
                                ];
    initial_measurement = [
                                          iris1.x(:,1); iris1.W(:,1); iris1_alone.force(:,1); iris1_alone.tau(:,1); ...
                                          iris2.x(:,1); iris2.W(:,1); iris2_alone.force(:,1); iris2_alone.tau(:,1)
                                       ];
    measurement_vec = [
                                        iris1.x; iris1.W; iris1_alone.force; iris1_alone.tau; ...
                                        iris2.x; iris2.W; iris2_alone.force; iris2_alone.tau
                                     ]; %+ 0.1*randn(length(initial_measurement),length(t_vec)); 
    
    %% ukf
    n=length(initial_state);      %number of state
    m=length(initial_measurement);     %number of measurement
    %std of process 
    q=[
          1e-3 1e-3 1e-3 ... %x  ------ iris1 
          1e-3 1e-3 1e-3 ... %v 
          1e-1 1e-1 1e-1 ... %a
          1e-3 1e-3 1e-3 ... %W
          1e-1 1e-1 1e-1 ... %dW
          1e-3 1e-3 1e-3 1e-3 ...%E
          1e-3 1e-3 1e-3 ... %x  ------ iris2 
          1e-3 1e-3 1e-3 ... %v 
          1e-1 1e-1 1e-1 ... %a
          1e-3 1e-3 1e-3 ... %W
          1e-1 1e-1 1e-1 ... %dW
          1e-3 1e-3 1e-3 1e-3 %E
         ];    
    %std of measurement
    r=[
         1e-3 1e-3 1e-3 ... %x ----- iris1
         1e-3 1e-3 1e-3 ... %W
         1e-3 1e-3 1e-3 ... %F
         1e-3 1e-3 1e-3 ... %tau
         1e-3 1e-3 1e-3 ... %x ----- iris2
         1e-3 1e-3 1e-3 ... %W
         1e-3 1e-3 1e-3 ... %F
         1e-3 1e-3 1e-3 ... %tau
        ];    
    Q=diag(q)*diag(q);       % covariance of process
    R=diag(r)*diag(r);        % covariance of measurement  
    % nonlinear state equations
    f=@(x,iris1,iris1_alone,iris2,iris2_alone,i) [
                                            % iris1            
                                            % x
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
                                            x(19);
                                            %
                                            % iris2            
                                            % x
                                            x(20)+x(23)*dt+0.5*x(26)*dt^2;
                                            x(21)+x(24)*dt+0.5*x(27)*dt^2;
                                            x(22)+x(25)*dt+0.5*x(28)*dt^2;   
                                            % v
                                            x(23)+x(26)*dt;
                                            x(24)+x(27)*dt;
                                            x(25)+x(28)*dt;
                                            % a
                                            x(26);
                                            x(27);
                                            x(28);
                                            % W
                                            x(29)+x(32)*dt;
                                            x(30)+x(33)*dt;
                                            x(31)+x(34)*dt;
                                            % dW
                                            x(32);
                                            x(33);
                                            x(34);
                                            % E
                                            x(35);
                                            x(36);
                                            x(37);
                                            x(38)
                                           ];
                                       
    % measurement equation
    h=@(x,iris1,iris1_alone,iris2,iris2_alone,i) [
                                            %iris1
                                            %x
                                            x(1);
                                            x(2);
                                            x(3);
                                            %W
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
                                             + cross([x(10); x(11); x(12)], iris1.J*[x(10); x(11); x(12)]);
                                            %iris2
                                            %x
                                            x(20);
                                            x(21);
                                            x(22);
                                            %W
                                            x(29);
                                            x(30);
                                            x(31);
                                            %F
                                             iris2.m*[x(26);x(27);x(28)] ... 
                                            - vec_ned_to_enu(iris2.m*iris2.g*iris2.e3 - iris2_alone.forceE(x(35:38),iris2_alone,i)...
                                            *reshape(iris2.R(:,i),3,3)*iris2.e3);
                                            %tau
                                             iris2.J*[x(32);x(33);x(34)] ...
                                             - iris2_alone.momentE(x(35:38),iris2_alone,i) ...
                                             + cross([x(29); x(30); x(31)], iris1.J*[x(29); x(30); x(31)])
                                           ];                               
    x=initial_state;           
    P = eye(n);                                            % initial state covraiance
    N= length(t_vec);                                  % total dynamic steps
    xV = zeros(n,N);                                   % allocate memory
    for k=1:N
      z = measurement_vec(:,k);                % measurments
      [x, P] = ukf(f,x,P,h,z,Q,R,iris1,iris1_alone,iris2,iris2_alone,k);            % ukf 
      xV(:,k) = x;                            % save estimate
    end
    % plot 
    estimation_plot(t_vec,true_state_vec,measurement_vec,xV,iris1_alone,iris2_alone);    
end
