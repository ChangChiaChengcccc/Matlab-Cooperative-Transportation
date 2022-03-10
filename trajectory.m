classdef trajectory
   methods
       function out = traj_generate(~, t)
           % xd, vd, b1d
           out = zeros(12, 1);
           % xd
           out(1) = 3*cos(0.8*t);
           out(2) = 3*sin(0.8*t);
           out(3) = 1;
           % vd
           out(4) = 3*(-1)*0.8*sin(0.8*t);
           out(5) = 3*1*0.8*cos(0.8*t);
           out(6) = 0;
           % ad
           out(7) = 3*(-1)*1*0.8*0.8*cos(0.8*t);
           out(8) = 3*1*(-1)*0.8*0.8*sin(0.8*t);
           out(9) = 0;
           % b1d
           out(10) = 1;
           out(11) = 0;
           out(12) = 0;
       end
   end
end
