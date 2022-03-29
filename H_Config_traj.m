function [tlist,qlist] = H_Config_traj(Puma,T0c,q0)
%%
tmove = 10;
tsplit = tmove/3;

Rct2 = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)]*[cos(-pi/2) -sin(-pi/2) 0;sin(-pi/2) cos(-pi/2) 0;0 0 1];
hp_s = [0 0.105 0.1]';
thHe = c2j_curve_j(Puma,Rct2,T0c,hp_s,q0);
Tvia1 = [Rct2,[0.4 0.4 0.1]'; 0 0 0 1];
Tvia2 = [Rct2,[0.8 -0.1 0.3]'; 0 0 0 1];
thvia1 = Puma.ikine(Tvia1,'q0',thHe);
thvia2 = Puma.ikine(Tvia2,'q0',thvia1);
%thvia3 = Puma.ikine(Tvia3,'q0',thvia2);
v_via = 0.5;
[q1list,~,~] = make_curve(2, 333,v_via,[thHe;thvia1],tsplit);
[q2list,~,~] = make_curve(2, 333,v_via,[thvia1;thvia2],tsplit);
[q3list,~,~] = make_curve(2, 334,v_via,[thvia2;zeros(1,6)],tsplit);
%[q4list,~,~] = make_curve(2, points2,v_via,[thvia3;thHs],tsplit);
qlist = [q1list;q2list;q3list];
tlist = (0:0.01:10-0.01)';
m=3;
%plot_robot_jspace(m,Puma,qlist)