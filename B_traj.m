function [eulneed,anglerr,errlist,tlist,qlist,qdlist] = B_traj(Puma,T0c,q0)
%% draw b
%
t_draw = 20;
B_length = 0.1+0.075*4+0.025*2*pi;
tb_1 = round(20/B_length*0.1);
tb_2 = round(20/B_length*0.075);
tb_3 = (t_draw-tb_1-4*tb_2)/2;
pointsp1 = 60;

pointtr1 = 60;
pointtr2 = 60;

%%
to = 1:pointsp1;
Btraj_p1 = [0.105 0 0.1]';
Btraj_p2 = [0.105 0 0]';
Btraj_p3 = [0.105 0.075 0]';
Btraj_p4 = [0.105 0.075 0.05]';
Btraj_sp1 = [0.105*ones(1,pointsp1);
            0.075+0.025*sin(pi/pointsp1.*to);
            0.025-0.025*cos(pi/pointsp1.*to)];
Btraj_p5 = [0.105 0 0.05]';
Btraj_p6 = [0.105 0.075 0.05]';
Btraj_p7 = [0.105 0.075 0.1]';
Btraj_p8 = [0.105 0 0.1]';
Btraj_sp2 = [0.105*ones(1,pointsp1);
            0.075+0.025*sin(pi/pointsp1.*to);
            0.075-0.025*cos(pi/pointsp1.*to)];

Rct = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)];
%%
thlist_p12 = c2j_curve_c(Puma,Rct,T0c,[Btraj_p1,Btraj_p2],pointtr1,q0);
thlist_p23 = c2j_curve_c(Puma,Rct,T0c,[Btraj_p2,Btraj_p3],pointtr2,thlist_p12(size(thlist_p12,1),:));
thlist_sp1 = c2j_curve_j(Puma,Rct,T0c,Btraj_sp1,thlist_p23(size(thlist_p23,1),:));
thlist_p45 = c2j_curve_c(Puma,Rct,T0c,[Btraj_p4,Btraj_p5],pointtr2,thlist_sp1(size(thlist_sp1,1),:));
thlist_p56 = c2j_curve_c(Puma,Rct,T0c,[Btraj_p5,Btraj_p6],pointtr2,thlist_p45(size(thlist_p45,1),:));
thlist_sp2 = c2j_curve_j(Puma,Rct,T0c,Btraj_sp2,thlist_p56(size(thlist_p56,1),:));
thlist_p78 = c2j_curve_c(Puma,Rct,T0c,[Btraj_p7,Btraj_p8],pointtr2,thlist_sp2(size(thlist_sp2,1),:));

alist1 = [10 12 12 8 8 8];
%alist2 = [2.5 3 3 2 2 2];
sptmlist = ones(1,pointsp1-1)*tb_3./(pointsp1-1);
t1tmlist = ones(1,pointtr1-1)*tb_1./(pointtr1-1);
t2tmlist = ones(1,pointtr2-1)*tb_2./(pointtr2-1);
[sp1alist,~,sp1t_joint,sp1t_mid,sp1v_mid] = parameter_prepare(sptmlist,alist1,thlist_sp1);
[sp2alist,~,sp2t_joint,sp2t_mid,sp2v_mid] = parameter_prepare(sptmlist,alist1,thlist_sp2);
[p12alist,~,p12t_joint,p12t_mid,p12v_mid] = parameter_prepare(t1tmlist,alist1,thlist_p12);
[p23alist,~,p23t_joint,p23t_mid,p23v_mid] = parameter_prepare(t2tmlist,alist1,thlist_p23);
[p45alist,~,p45t_joint,p45t_mid,p45v_mid] = parameter_prepare(t2tmlist,alist1,thlist_p45);
[p56alist,~,p56t_joint,p56t_mid,p56v_mid] = parameter_prepare(t2tmlist,alist1,thlist_p56);
[p78alist,~,p78t_joint,p78t_mid,p78v_mid] = parameter_prepare(t2tmlist,alist1,thlist_p78);
rate=0.01;
[sp1tlist,sp1mneed,sp1utneed] = timeprepare(sp1t_joint,sp1t_mid,rate);
[sp2tlist,sp2mneed,sp2utneed] = timeprepare(sp2t_joint,sp2t_mid,rate);
[p12tlist,p12mneed,p12utneed] = timeprepare(p12t_joint,p12t_mid,rate);
[p23tlist,p23mneed,p23utneed] = timeprepare(p23t_joint,p23t_mid,rate);
[p45tlist,p45mneed,p45utneed] = timeprepare(p45t_joint,p45t_mid,rate);
[p56tlist,p56mneed,p56utneed] = timeprepare(p56t_joint,p56t_mid,rate);
[p78tlist,p78mneed,p78utneed] = timeprepare(p78t_joint,p78t_mid,rate);

[p12qlist,p12qdlist,~] = jointinter(p12alist,p12t_joint,p12t_mid,p12v_mid,thlist_p12',p12mneed,p12utneed);
[p23qlist,p23qdlist,~] = jointinter(p23alist,p23t_joint,p23t_mid,p23v_mid,thlist_p23',p23mneed,p23utneed);
[p45qlist,sp1qdlist,~] = jointinter(p45alist,p45t_joint,p45t_mid,p45v_mid,thlist_p45',p45mneed,p45utneed);
[p56qlist,p45qdlist,~] = jointinter(p56alist,p56t_joint,p56t_mid,p56v_mid,thlist_p56',p56mneed,p56utneed);
[p78qlist,p56qdlist,~] = jointinter(p78alist,p78t_joint,p78t_mid,p78v_mid,thlist_p78',p78mneed,p78utneed);
[sp1qlist,sp2qdlist,~]= jointinter(sp1alist,sp1t_joint,sp1t_mid,sp1v_mid,thlist_sp1',sp1mneed,sp1utneed);
[sp2qlist,p78qdlist,~]= jointinter(sp2alist,sp2t_joint,sp2t_mid,sp2v_mid,thlist_sp2',sp2mneed,sp2utneed);
%[sp1qlist,sp1qdlist,sp1qddlist] = make_curve(pointsp1, pointsp2, v_via,thlist_sp1,tb_3);
%[sp2qlist,sp2qdlist,sp2qddlist] = make_curve(pointsp1, pointsp2, v_via,thlist_sp2,tb_3);

tlist = [p12tlist;p23tlist+tb_1;sp1tlist+tb_1+tb_2;p45tlist+tb_1+tb_2+tb_3;p56tlist+tb_1+2*tb_2+tb_3;sp2tlist+tb_1+3*tb_2+tb_3;p78tlist+tb_1+3*tb_2+2*tb_3];
qlist = [p12qlist;p23qlist;sp1qlist;p45qlist;p56qlist;sp2qlist;p78qlist];
qdlist = [p12qdlist;p23qdlist;sp1qdlist;p45qdlist;p56qdlist;sp2qdlist;p78qdlist];
%qddlist = [p12qddlist;p23qddlist;sp1qddlist;p45qddlist;p56qddlist;sp2qddlist;p78qddlist];
m=40;
%%
for i = 1:tb_3*100

    Btraj_sp2 = [0.105;
            0.075+0.025*sin(pi/(tb_3*100).*i);
            0.075-0.025*cos(pi/(tb_3*100).*i)];
    Btraj_sp1 = [0.105;
            0.075+0.025*sin(pi/(tb_3*100).*i);
            0.025-0.025*cos(pi/(tb_3*100).*i)];
    Tsp1 = T0c*[Rct,Btraj_sp1;0 0 0 1];
    Tsp2 = T0c*[Rct,Btraj_sp2;0 0 0 1];
    realTsp1(:,:,i) = Tsp1;
    realTsp2(:,:,i) = Tsp2;
end
realTp12 = use_ctraj([Btraj_p1,Btraj_p2],Rct,T0c,tb_1*100);
realTp23 = use_ctraj([Btraj_p2,Btraj_p3],Rct,T0c,tb_2*100);
realTp45 = use_ctraj([Btraj_p4,Btraj_p5],Rct,T0c,tb_2*100);
realTp56 = use_ctraj([Btraj_p5,Btraj_p6],Rct,T0c,tb_2*100);
realTp78 = use_ctraj([Btraj_p7,Btraj_p8],Rct,T0c,tb_2*100);
realT = cat(3,realTp12,realTp23,realTsp1,realTp45,realTp56,realTsp2,realTp78);
errlist = [];
anglerr = [];
eulneed = [];
for lp = 1:size(qlist,1)
    T = Puma.fkine(qlist(lp,:)).T;
    err = (T(1:3,4)-realT(1:3,4,lp))'*(T(1:3,4)-realT(1:3,4,lp));
    errlist = [errlist err];
    eul1 = tr2eul(T);
    eul2 = tr2eul(realT(:,:,lp));
    anglerr = [anglerr;eul1-eul2];
    eulneed = [eulneed;eul1];
end



%plot_robot_jspace(m,Puma,qlist)
%%
%figure
%plot(tlist(:,1),qlist(:,1),'*')
%hold on
%plot(tlist(:,2),qlist(:,2),'*')
%plot(tlist(:,3),qlist(:,3),'*')
%plot(tlist(:,4),qlist(:,4),'*')
%plot(tlist(:,5),qlist(:,5),'*')
%plot(tlist(:,6),qlist(:,6),'*')

