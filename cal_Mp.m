function [F1_body, F2_body, Mp] = cal_Mp(iris1,iris2,iris1_alone,iris2_alone,i)
    % matrix from (-F)ned to Mp
    L1 = [0 0 iris1.x(1,1); 
              0 0 0; 
              -iris1.x(1,1) 0 0];
          
    L2 = [0 0 iris2.x(1,1);
              0 0 0;
              -iris2.x(1,1) 0 0];
    % calculate F1_body,F2_body,Mp
    F1_body = inv(reshape(iris1.R(:,i),3,3))*vec_enu_to_ned(-iris1_alone.force(:,i));
    F2_body = inv(reshape(iris2.R(:,i),3,3))*vec_enu_to_ned(-iris2_alone.force(:,i));
    
    Mp1 = L1* inv(reshape(iris1.R(:,i),3,3))*vec_enu_to_ned(-iris1_alone.force(:,i));
    Mp2 = L2* inv(reshape(iris2.R(:,i),3,3))*vec_enu_to_ned(-iris2_alone.force(:,i));      
    Mp = Mp1 + Mp2;
end