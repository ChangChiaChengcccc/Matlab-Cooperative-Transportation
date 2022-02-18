function a_enu_wdot = dvdW(object,iter,f_M)
        a_enu_wdot = zeros(6, 1);
        a_ned = object.g*object.e3 - (f_M(1)/object.m)*reshape(object.R(:, iter),3,3)*object.e3;
        a_enu_wdot(1:3) = vec_ned_to_enu(a_ned);
        a_enu_wdot(4:6) = object.J\(-vec_cross(object.W(:, iter), object.J*object.W(:, iter)) + f_M(2:4));
end