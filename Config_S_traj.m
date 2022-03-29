function [tlist,qlist]= Config_S_traj(Puma,T0c)
%%
tmove = 10;
Rct = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)];
Tcon = Puma.fkine([0 0 0 0 0 0]).T;


pSs = [0.025,0.025,0.105]';
Tvia = [Rct, [0.5;-0.105;0.4];
        0 0 0 1;];
thvia = Puma.ikine(Tvia,'q0',[0 0 0 0 0 0]);
thSs = c2j_curve_j(Puma,Rct,T0c,pSs,thvia);

[q1list,qd1list,~] = make_curve(2,500,0,[[0 0 0 0 0 0];thvia],5);
[q2list,qd2list,~] = make_curve(2,500,0,[thvia;thSs],5);
qlist = [q1list;q2list];
qdlist = [qd1list;qd2list];
tlist = (0:0.01:10-0.01)';
plot(qlist)
cube = [0.4,0.1,0;
        0.5,0.1,0;
        0.5,0.2,0;
        0.4,0.2,0;
        0.4,0.1,0.1;
        0.5,0.1,0.1;
        0.5,0.2,0.1;
        0.4,0.2,0.1;];

plot3(cube(:,1),cube(:,2),cube(:,3),'b*')
hold on
%plot_robot_jspace(m,Puma,qlist)
