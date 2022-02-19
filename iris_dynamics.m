function  out = iris1_dynamics(q,sys,iter)
    r_qs = q.x - sys.x(:, 1);
    r_qs_ned = vec_enu_to_ned(r_qs);
    out = sys.x(:,iter) + vec_ned_to_enu(reshape(sys.R(:,iter),3,3)*r_qs_ned);
end