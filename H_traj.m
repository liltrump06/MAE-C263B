function [eulneed,anglerr,errlist,tlist,qlist,qdlist] = H_traj(Puma,T0c,q0)
%%

%% draw h
%
t_draw = 20;
B_length = 3*0.1+2*0.05;
th_1 = round(20/B_length*0.1);
th_2 = (20-3*th_1)/2;

pointtr1 = th_1*14;
pointtr2 = th_2*14;
Htraj_p1 = [0.1 0.105 0.1]';
Htraj_p2 = [0.1 0.105 0]';
Htraj_p3 = [0.1 0.105 0.05]';
Htraj_p4 = [0 0.105 0.05]';
Htraj_p5 = [0 0.105 0]';
Htraj_p6 = [0 0.105 0.1]';
Rct = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)]*[cos(-pi/2) -sin(-pi/2) 0;sin(-pi/2) cos(-pi/2) 0;0 0 1];

%%
thlist_p12 = c2j_curve_c(Puma,Rct,T0c,[Htraj_p1,Htraj_p2],pointtr1,q0);
thlist_p23 = c2j_curve_c(Puma,Rct,T0c,[Htraj_p2,Htraj_p3],pointtr2,thlist_p12(size(thlist_p12,1),:));
thlist_p34 = c2j_curve_c(Puma,Rct,T0c,[Htraj_p3,Htraj_p4],pointtr1,thlist_p23(size(thlist_p23,1),:));
thlist_p45 = c2j_curve_c(Puma,Rct,T0c,[Htraj_p4,Htraj_p5],pointtr2,thlist_p34(size(thlist_p34,1),:));
thlist_p56 = c2j_curve_c(Puma,Rct,T0c,[Htraj_p5,Htraj_p6],pointtr1,thlist_p45(size(thlist_p45,1),:));
%%
rate = 0.01;
alist1 = [10 12 12 8 8 8]./2;
[p12alist,~,p12t_joint,p12t_mid,p12v_mid] = parameter_prepare(th_1/(pointtr1-1),alist1,thlist_p12);
[p23alist,~,p23t_joint,p23t_mid,p23v_mid] = parameter_prepare(th_2/(pointtr2-1),alist1,thlist_p23);
[p34alist,~,p34t_joint,p34t_mid,p34v_mid] = parameter_prepare(th_1/(pointtr1-1),alist1,thlist_p34);
[p45alist,~,p45t_joint,p45t_mid,p45v_mid] = parameter_prepare(th_2/(pointtr2-1),alist1,thlist_p45);
[p56alist,~,p56t_joint,p56t_mid,p56v_mid] = parameter_prepare(th_1/(pointtr1-1),alist1,thlist_p56);
[p12tlist,p12mneed,p12utneed] = timeprepare(p12t_joint,p12t_mid,rate);
[p23tlist,p23mneed,p23utneed] = timeprepare(p23t_joint,p23t_mid,rate);
[p34tlist,p34mneed,p34utneed] = timeprepare(p34t_joint,p34t_mid,rate);
[p45tlist,p45mneed,p45utneed] = timeprepare(p45t_joint,p45t_mid,rate);
[p56tlist,p56mneed,p56utneed] = timeprepare(p56t_joint,p56t_mid,rate);
[p12qlist,p12qdlist,~] = jointinter(p12alist,p12t_joint,p12t_mid,p12v_mid,thlist_p12',p12mneed,p12utneed);
[p23qlist,p23qdlist,~] = jointinter(p23alist,p23t_joint,p23t_mid,p23v_mid,thlist_p23',p23mneed,p23utneed);
[p34qlist,p34qdlist,~] = jointinter(p34alist,p34t_joint,p34t_mid,p34v_mid,thlist_p34',p34mneed,p34utneed);
[p45qlist,p45qdlist,~] = jointinter(p45alist,p45t_joint,p45t_mid,p45v_mid,thlist_p45',p45mneed,p45utneed);
[p56qlist,p56qdlist,~] = jointinter(p56alist,p56t_joint,p56t_mid,p56v_mid,thlist_p56',p56mneed,p56utneed);
tlist = [p12tlist;p23tlist+th_1;p34tlist+th_1+th_2;p45tlist+2*th_1+th_2;p56tlist+2*th_1+2*th_2];
qlist = [p12qlist;p23qlist;p34qlist;p45qlist;p56qlist];
qdlist = [p12qdlist;p23qdlist;p34qdlist;p45qdlist;p56qdlist];
%qddlist = [p12qddlist;p23qddlist;p34qddlist;p45qddlist;p56qddlist];
m=5;
%%
realTp12 = use_ctraj([Htraj_p1,Htraj_p2],Rct,T0c,th_1*100);
for i = 1:th_2*100
    Htraj_p23 = [0.1 0.105 i/(th_2*100)*0.05]';
    Htraj_p45 = [0 0.105 0.05-i/(th_2*100)*0.05]';
    Tsp1 = T0c*[Rct,Htraj_p23;0 0 0 1];
    Tsp2 = T0c*[Rct,Htraj_p45;0 0 0 1];

    realTp23(:,:,i) = Tsp1;
    realTp45(:,:,i) = Tsp2;
end
%realTp23 = use_ctraj([Htraj_p2,Htraj_p3],Rct,T0c,th_2*100);
realTp34 = use_ctraj([Htraj_p3,Htraj_p4],Rct,T0c,th_1*100);
%realTp45 = use_ctraj([Htraj_p4,Htraj_p5],Rct,T0c,th_2*100);
realTp56 = use_ctraj([Htraj_p5,Htraj_p6],Rct,T0c,th_1*100);
realT = cat(3,realTp12,realTp23,realTp34,realTp45,realTp56);
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