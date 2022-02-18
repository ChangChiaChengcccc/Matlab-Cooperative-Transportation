function sys = sys_inertia(iris1, iris2, payload)
    sys = multirotor_dynamics;
    sys.m = iris1.m + iris2.m + payload.m;
end