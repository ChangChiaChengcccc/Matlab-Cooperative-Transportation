function out = test_fM(sys,q1,q2,iris1_alone,iris2_alone)
    step = sys.sim_t / sys.dt + 1;
    out = zeros(4,step);
    
    r1_qs = q1.x(:, 1) - sys.x(:, 1); %enu
    r1_qs_ned = vec_enu_to_ned(r1_qs); %ned;
    
    r2_qs = q2.x(:, 1) - sys.x(:, 1); %enu
    r2_qs_ned = vec_enu_to_ned(r2_qs); %ned;
    
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
    for i  = 1:step
    u = [iris1_alone.force_moment(:,i); iris2_alone.force_moment(:,i)];
    sys_fM = A*u;
    difference  = (sys_fM - sys.force_moment(:,i));
    out(:,i) = abs(difference) <= [1e-7;1e-7;1e-7;1e-7];
    end
end