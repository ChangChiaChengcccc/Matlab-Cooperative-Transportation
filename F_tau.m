function  [F, tau]= F_tau(object,object_alone,iter)
    F = object.m*object.a(:,iter) - object_alone.m*object_alone.a(:,iter);
    tau = object.J*object.dW(:,iter) - object_alone.J*object_alone.dW(:,iter);
end