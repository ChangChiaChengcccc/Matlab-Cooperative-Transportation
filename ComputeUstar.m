function Ustar = ComputeUstar(q1,q2,sys,sys_fM)
    
    %A = zeros(4,8);
    %H = zeros(8,8);
    r1_qs = q1.x - sys.x(:, 1); %enu
    r1_qs_ned = vec_enu_to_ned(r1_qs); %ned;
    
    r2_qs = q2.x - sys.x(:, 1); %enu
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
   %% H
    H = 2^0.5*eye(8);
    
   %% u*
    H_inv_2 = inv(H)*inv(H);
    translator = H_inv_2*A'/(A*H_inv_2*A');
    Ustar = translator*sys_fM
    
    %% test
     %original_fM = A*Ustar
end