function thsplist = c2j_curve_j(robot,Rct2,T0c,traj,q0)
thsplist=[];
thsp = zeros(1,6);

for i = 1:size(traj,2)
    Tcpsp = [Rct2,traj(:,i);
             0 0 0 1];
    T0sp=T0c*Tcpsp;
    if i == 1
        thsp = robot.ikine(T0sp,'rlimit',1000,'q0',q0);
        thsplist = [thsplist ;thsp];
    else
        thsp = robot.ikine(T0sp,'rlimit',1000,'q0',thsp);
        thsplist = [thsplist ;thsp];
    end
end
end
