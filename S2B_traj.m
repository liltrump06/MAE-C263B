function [tlist,qlist] = S2B_traj(Puma,T0c,q0)
%%
tmove = 10;
points2 = 100;

Rct1 = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)];
sp_e = [0.075,0.075,0.1]';
Rct2 = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)];
bp_s = [0.1 0 0.1]';
thSe = c2j_curve_j(Puma,Rct1,T0c,sp_e,q0);
Tvia1 = [Rct1,[0.575,0.275,0.3]';0 0 0 1];
Tvia2 = [Rct1,[0.575,0.375,0.3]';0 0 0 1];
%Tvia3 = [Rct2,[0.7,0.2,0.1]';0 0 0 1];
thvia1 = Puma.ikine(Tvia1,'q0',thSe);
thvia2 = Puma.ikine(Tvia2,'q0',thvia1);
thBs = c2j_curve_j(Puma,Rct2,T0c,bp_s,thvia2);
%thvia3 = Puma.ikine(Tvia3,'q0',thvia2);
v_via = 0.5;
[q1list,~,~] = make_curve(2, 333,v_via,[thSe;thvia1],2.5);
[q2list,~,~] = make_curve(2, 333,v_via,[thvia1;thvia2],2.5);
[q3list,~,~] = make_curve(2, 334,v_via,[thvia2;thBs],2.5);
%[q4list,~,~] = make_curve(2, points2,v_via,[thvia3;thBs],2.5);
qlist = [q1list;q2list;q3list];
tlist = (0:0.01:10-0.01)';
hold on

