function  [x_q, v_q, a_q] = iris_dynamics(q,sys,iter)
    %x 2 terms
    r_qs = q.x(:, 1) - sys.x(:, 1); %enu
    r_qs_ned = vec_enu_to_ned(r_qs); %ned;
    x_q = sys.x(:,iter) + vec_ned_to_enu(reshape(sys.R(:,iter),3,3)*r_qs_ned); %enu
    
    %v 2 terms
    v_ned = cross(sys.W(:,iter),r_qs_ned);
    v_q = sys.v(:,iter) + vec_ned_to_enu(v_ned);
    
    %a 3 terms
    a1_ned = cross(sys.dW(:,iter),r_qs_ned);
    a2_ned = cross(sys.W(:,iter),cross(sys.W(:,iter),r_qs_ned));
    a_q = sys.a(:,iter) + vec_ned_to_enu(a1_ned) + vec_ned_to_enu(a2_ned);
end