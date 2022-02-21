function [a_enu,wdot] = dvdW(object,iter,f_M)
        a_ned = object.g*object.e3 - (f_M(1)/object.m)*reshape(object.R(:, iter),3,3)*object.e3;
        a_enu = vec_ned_to_enu(a_ned);
        wdot = object.J\(-vec_cross(object.W(:, iter), object.J*object.W(:, iter)) + f_M(2:4));
end