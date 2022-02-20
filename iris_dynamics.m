function  out = iris_dynamics(q,sys,iter)
    out = zeros(9,1);
    %x 2 terms
    r_qs = q.x - sys.x(:, 1); %enu
    r_qs_ned = vec_enu_to_ned(r_qs); %ned;
    out(1:3) = sys.x(:,iter) + vec_ned_to_enu(reshape(sys.R(:,iter),3,3)*r_qs_ned); %enu
    
    %v 2 terms
    v_ned = cross(sys.W(:,iter),r_qs_ned);
    out(4:6) = sys.v(:,iter) + vec_ned_to_enu(v_ned);
    
    %a 3 terms
    a1_ned = cross(sys.dW(:,iter),r_qs_ned);
    a2_ned = cross(sys.W(:,iter),cross(sys.W(:,iter),r_qs_ned));
    out(7:9) = sys.a(:,iter) + vec_ned_to_enu(a1_ned) + vec_ned_to_enu(a2_ned);
end