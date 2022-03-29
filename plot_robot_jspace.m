function plot_robot_jspace(m,robot,Q)
axis([-1,1,-1,1,-1,1])
for i = 1 : size(Q,1)/m
    q = Q(i*m-(m-1),:);
    T = robot.fkine(q);
    robot.plot(q);
    [~,FT] = tr2rt(T);
    plot3(FT(1),FT(2),FT(3),'r*')
    hold on
end