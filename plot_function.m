function plot_function(iris1,iris2,system)
    % % overall path
    % plot3(system.x(1, :),system.x(2, :),system.x(3, :),...
    %          iris1.x(1,:),iris1.x(2,:),iris1.x(3,:), ...
    %          iris2.x(1,:),iris2.x(2,:),iris2.x(3,:));
    % grid on

    %% system
    % path
    plot3(system.x(1, 24000:30001),system.x(2, 24000:30001),system.x(3, 24000:30001))
    hold on
    grid on
    zlim([0 1.5])
    % v vector
    quiver3(system.x(1, 24000),system.x(2, 24000),system.x(3, 24000),system.v(1, 24000),system.v(2, 24000),system.v(3, 24000))
    quiver3(system.x(1, 25120),system.x(2, 25120),system.x(3, 25120),system.v(1, 25120),system.v(2, 25120),system.v(3, 25120))
    quiver3(system.x(1, 26690),system.x(2, 26690),system.x(3, 26690),system.v(1, 26690),system.v(2, 26690),system.v(3, 26690))
    % a vector
    quiver3(system.x(1, 24000),system.x(2, 24000),system.x(3, 24000),system.a(1, 24000),system.a(2, 24000),system.a(3, 24000))
    quiver3(system.x(1, 25120),system.x(2, 25120),system.x(3, 25120),system.a(1, 25120),system.a(2, 25120),system.a(3, 25120))
    quiver3(system.x(1, 26690),system.x(2, 26690),system.x(3, 26690),system.a(1, 26690),system.a(2, 26690),system.a(3, 26690))
    
   %% iris1
    % path
    plot3(iris1.x(1,24000:30001),iris1.x(2,24000:30001),iris1.x(3,24000:30001))
    % v vector
    quiver3(iris1.x(1,24000),iris1.x(2,24000),iris1.x(3,24000),iris1.v(1,24000),iris1.v(2,24000),iris1.v(3,24000))
    quiver3(iris1.x(1,25120),iris1.x(2,25120),iris1.x(3,25120),iris1.v(1,25120),iris1.v(2,25120),iris1.v(3,25120))
    quiver3(iris1.x(1,26690),iris1.x(2,26690),iris1.x(3,26690),iris1.v(1,26690),iris1.v(2,26690),iris1.v(3,26690))
    % a vector
    quiver3(iris1.x(1,24000),iris1.x(2,24000),iris1.x(3,24000),iris1.a(1,24000),iris1.a(2,24000),iris1.a(3,24000))
    quiver3(iris1.x(1,25120),iris1.x(2,25120),iris1.x(3,25120),iris1.a(1,25120),iris1.a(2,25120),iris1.a(3,25120))
    quiver3(iris1.x(1,26690),iris1.x(2,26690),iris1.x(3,26690),iris1.a(1,26690),iris1.a(2,26690),iris1.a(3,26690))
    
   %% iris2
    % path
    plot3(iris2.x(1,24000:30001),iris2.x(2,24000:30001),iris2.x(3,24000:30001))
    % v vector
    quiver3(iris2.x(1,24000),iris2.x(2,24000),iris2.x(3,24000),iris2.v(1,24000),iris2.v(2,24000),iris2.v(3,24000))
    quiver3(iris2.x(1,25120),iris2.x(2,25120),iris2.x(3,25120),iris2.v(1,25120),iris2.v(2,25120),iris2.v(3,25120))
    quiver3(iris2.x(1,26690),iris2.x(2,26690),iris2.x(3,26690),iris2.v(1,26690),iris2.v(2,26690),iris2.v(3,26690))
    % a vector
    quiver3(iris2.x(1,24000),iris2.x(2,24000),iris2.x(3,24000),iris2.a(1,24000),iris2.a(2,24000),iris2.a(3,24000))
    quiver3(iris2.x(1,25120),iris2.x(2,25120),iris2.x(3,25120),iris2.a(1,25120),iris2.a(2,25120),iris2.a(3,25120))
    quiver3(iris2.x(1,26690),iris2.x(2,26690),iris2.x(3,26690),iris2.a(1,26690),iris2.a(2,26690),iris2.a(3,26690))

end