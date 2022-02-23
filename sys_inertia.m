function sys= sys_inertia(q1,q2,p,sys)
    
    % transfer frame to system body frame
    q1_x_ned = vec_enu_to_ned(q1.x(:,1));
    q2_x_ned = vec_enu_to_ned(q2.x(:,1));
    p_x_ned = vec_enu_to_ned(p.x);
    
    % central mass of the system
    sys_m = q1.m + q2.m +p.m;
    x_com = (q1_x_ned(1)*q1.m+q2_x_ned(1)*q2.m+p_x_ned(1) *p.m) / sys_m;
    y_com = (q1_x_ned(2)*q1.m+q2_x_ned(2)*q2.m+p_x_ned(2) *p.m) / sys_m;
    z_com = (q1_x_ned(3)*q1.m+q2_x_ned(3)*q2.m+p_x_ned(3) *p.m) / sys_m;
    
    % rotation inertia of the system
    q1_Jxx = q1.J(1,1);
    q1_Jyy = q1.J(2,2);
    q1_Jzz = q1.J(3,3);
    
    q2_Jxx = q2.J(1,1);
    q2_Jyy = q2.J(2,2);
    q2_Jzz = q2.J(3,3);
    
    p_Jxx = p.J(1,1);
    p_Jyy = p.J(2,2);
    p_Jzz = p.J(3,3);
    
    % Jxx = Jg,xx + m*(y^2+z^2)
    Jxx_system = (q1_Jxx + q1.m*((q1_x_ned(3) - z_com)^2 +(q1_x_ned(2) - y_com)^2)) + (q2_Jxx + q2.m*((q2_x_ned(3) - z_com)^2 ...
                            + (q2_x_ned(2) - y_com)^2)) + (p_Jxx + p.m*((p_x_ned(3) - z_com)^2 + (p_x_ned(2) - y_com)^2));
    Jyy_system = (q1_Jyy + q1.m*((q1_x_ned(3) - z_com)^2 +(q1_x_ned(1) - x_com)^2)) + (q2_Jyy + q2.m*((q2_x_ned(3) - z_com)^2 ...
                            + (q2_x_ned(1) - x_com)^2)) + (p_Jyy + p.m*((p_x_ned(3) - z_com)^2 + (p_x_ned(1) - x_com)^2));         
    Jzz_system = (q1_Jzz + q1.m*((q1_x_ned(2) - y_com)^2 +(q1_x_ned(1) - x_com)^2)) + (q2_Jzz + q2.m*((q2_x_ned(2) - y_com)^2 ...
                            + (q2_x_ned(1) - x_com)^2)) + (p_Jzz + p.m*((p_x_ned(2) - y_com)^2 + (p_x_ned(1) - x_com)^2));   
    % Jxy = Jyx = Jg,xy - m*(x*y)
    Jxy_system = -q1.m*(q1_x_ned(2) - y_com)*(q1_x_ned(1) - x_com)-q2.m*(q2_x_ned(2) - y_com)*(q2_x_ned(1) - x_com) ...
                           -p.m*(p_x_ned(2) - y_com)*(p_x_ned(1) - x_com);
    Jyx_system = Jxy_system;
    
    Jxz_system = -q1.m*(q1_x_ned(1) - x_com)*(q1_x_ned(3) - z_com)-q2.m*(q2_x_ned(1) - x_com)*(q2_x_ned(3) - z_com) ...
                           -p.m*(p_x_ned(1) - x_com)*(p_x_ned(3) - z_com);
    Jzx_system = Jxz_system;
    
    Jyz_system = -q1.m*(q1_x_ned(2) - y_com)*(q1_x_ned(3) - z_com)-q2.m*(q2_x_ned(2) - y_com)*(q2_x_ned(3) - z_com) ...
                           -p.m*(p_x_ned(2) - y_com)*(p_x_ned(3) - z_com);
    Jzy_system = Jyz_system;                   
                    
    sys.m =  sys_m;
    sys.J = [Jxx_system, Jxy_system, Jxz_system;
                 Jyx_system, Jyy_system, Jyz_system;
                 Jzx_system, Jzy_system, Jzz_system];
end