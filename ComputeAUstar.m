function original_fM = ComputeAUstar(q1,q2,sys,q1_alone,q2_alone,i)
    
    %A = zeros(4,8);
    %H = zeros(8,8);
    r1_qs = q1.x(:, 1) - sys.x(:, 1); %enu
    r1_qs_ned = vec_enu_to_ned(r1_qs); %ned;
    
    r2_qs = q2.x(:, 1) - sys.x(:, 1); %enu
    r2_qs_ned = vec_enu_to_ned(r2_qs); %ned;
   %% A
    % iris1 is [0, 0.6, 0] in system body-fixed frame
    T = 0; % relative angle    
    A1 = [                     1,         0,          0,   0;
                 r1_qs_ned(2), cos(T),  -sin(T),   0;
                -r1_qs_ned(1),  sin(T),  cos(T),   0;
                                    0,         0,          0,   1];
    % iris2 is [0, -0.6, 0] in system body-fixed frame
    A2 = [                     1,         0,          0,   0;
                 r2_qs_ned(2), cos(T),  -sin(T),   0;
                -r2_qs_ned(1),  sin(T),  cos(T),   0;
                                    0,         0,          0,   1];
    A = [A1 A2];
   
   %% original_fM
    U = [q1_alone.fault_force_moment(:,i); q2_alone.fault_force_moment(:,i)]; 
    original_fM = A*U;
end