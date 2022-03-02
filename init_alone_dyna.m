function object = init_alone_dyna(object,system)
    object.m = 1.55;
    object.J = [0.0347563, 0, 0;
                    0, 0.0458929, 0;
                    0, 0, 0.0977];
    object.x = zeros(3, length(system.t));
    object.v = zeros(3, length(system.t));
    object.a = zeros(3, length(system.t));
    object.R = zeros(9, length(system.t));
    object.W = zeros(3, length(system.t));
    object.dW = zeros(3, length(system.t));
    object.force_moment = zeros(4, length(system.t));
    object.F = zeros(3, length(system.t));
    object.tau = zeros(3, length(system.t));
end