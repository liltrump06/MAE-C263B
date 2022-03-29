function [tlist,qlist]= B2H_traj(Puma,T0c,q0)
%%
tmove = 10;
tsplit = tmove/4;
points2 = 250;
Rct1 = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)];
bp_e = [0.105,0,0.1]';
Rct2 = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)]*[cos(-pi/2) -sin(-pi/2) 0;sin(-pi/2) cos(-pi/2) 0;0 0 1];
hp_s = [0.1 0.105 0.1]';
thBe = c2j_curve_j(Puma,Rct1,T0c,bp_e,q0);

Tvia1 = [Rct1,[0.55,0.15,0.1]';0 0 0 1];
Tvia2 = [Rct1,[0.55 0.2 0.2]';0 0 0 1];
Tvia3 = [Rct1,[0.45 0.55 0.2]';0 0 0 1];
thvia1 = Puma.ikine(Tvia1,'q0',thBe);
thvia2 = Puma.ikine(Tvia2,'q0',thvia1);
thvia3 = Puma.ikine(Tvia3,'q0',thvia2);
thHs = c2j_curve_j(Puma,Rct2,T0c,hp_s,[0 0 0 0 0 0]);
v_via = 0.5;
[q1list,~,~] = make_curve(2, points2,v_via,[thBe;thvia1],tsplit);
[q2list,~,~] = make_curve(2, points2,v_via,[thvia1;thvia2],tsplit);
[q3list,~,~] = make_curve(2, points2,v_via,[thvia2;thvia3],tsplit);
[q4list,~,~] = make_curve(2, points2,v_via,[thvia3;thHs],tsplit);
qlist = [q1list;q2list;q3list;q4list];
tlist = (0:0.01:10-0.01)';
m=2;
%plot_robot_jspace(m,Puma,qlist)
