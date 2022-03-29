function [eulneed,anglerr,errlist,tlist,qlist,qdlist] =S_traj(Puma,T0c,q0)

%%
t_draw = 20;
S_length = 0.025*2*2*pi+0.05*sqrt(2);
ts_1 = round(t_draw/S_length*0.025*2*pi);
ts_2 = t_draw-2*ts_1;
pointsp1 = ts_1*30;
pointtr2 = ts_2*15;
to = 1:pointsp1;
Straj_sp1 = [0.05-0.025*cos(pi/pointsp1.*to);
             0.025-0.025*sin(pi/pointsp1.*to);
             0.105*ones(1,pointsp1)];
Straj_sp2 = [0.05-0.025*cos(pi/pointsp1.*to);
             0.075+0.025*sin(pi/pointsp1.*to);
             0.105*ones(1,pointsp1)];
Straj_p1 = [0.075,0.025,0.105]';
Straj_p2 = [0.025,0.075,0.105]';

Rct = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)]*[cos(pi/2) 0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)];
%%
rate = 0.01;

thlist_sp1 = c2j_curve_j(Puma,Rct,T0c,Straj_sp1,q0);
thlist_p12 = c2j_curve_c(Puma,Rct,T0c,[Straj_p1,Straj_p2],pointtr2,thlist_sp1(size(thlist_sp1,1),:));
thlist_sp2 = c2j_curve_j(Puma,Rct,T0c,Straj_sp2,thlist_p12(size(thlist_p12,1),:));
alist1 = [10 12 12 8 8 8];

%%
[sp1alist,~,sp1t_joint,sp1t_mid,sp1v_mid] = parameter_prepare(ts_1/(pointsp1-1),alist1,thlist_sp1);
[sp2alist,~,sp2t_joint,sp2t_mid,sp2v_mid] = parameter_prepare(ts_1/(pointsp1-1),alist1,thlist_sp2);
[p12alist,~,p12t_joint,p12t_mid,p12v_mid] = parameter_prepare(ts_2/(pointtr2-1),alist1,thlist_p12);
[sp1tlist,sp1mneed,sp1utneed] = timeprepare(sp1t_joint,sp1t_mid,rate);
[sp2tlist,sp2mneed,sp2utneed] = timeprepare(sp2t_joint,sp2t_mid,rate);
[p12tlist,p12mneed,p12utneed] = timeprepare(p12t_joint,p12t_mid,rate);
[sp1qlist,sp1qdlist,~]= jointinter(sp1alist,sp1t_joint,sp1t_mid,sp1v_mid,thlist_sp1',sp1mneed,sp1utneed);
[sp2qlist,p12qdlist,~]= jointinter(sp2alist,sp2t_joint,sp2t_mid,sp2v_mid,thlist_sp2',sp2mneed,sp2utneed);
[p12qlist,sp2qdlist,~]= jointinter(p12alist,p12t_joint,p12t_mid,p12v_mid,thlist_p12',p12mneed,p12utneed);

%[tlist,qlist,qdlist,qddlist] = pre_jointinter(sp1alist,sp1t_joint,sp1t_mid,sp1v_mid,thlist_sp1',sp1,sp2);
%[sp2tlist,sp2qlist,sp2qdlist,sp2qddlist]= jointinter(sp2alist,sp2t_joint,sp2t_mid,sp2v_mid,thlist_sp2',sp1,sp2);
%[p12tlist,p12qlist,p12qdlist,p12qddlist]= jointinter(p12alist,p12t_joint,p12t_mid,p12v_mid,thlist_p12',sp1,sp2);

tlist = [sp1tlist;p12tlist+ts_1;sp2tlist+ts_1+ts_2];
qlist = [sp1qlist;p12qlist;sp2qlist];
qdlist = [sp1qdlist;p12qdlist;sp2qdlist];
%qddlist = [sp1qddlist;p12qddlist;sp2qddlist];
m=20;
%%
for i = 1:ts_1*100

    Straj_sp1 = [0.05-0.025*cos(pi/(ts_1*100).*i);
             0.025-0.025*sin(pi/(ts_1*100).*i);
             0.105];
    Straj_sp2 = [0.05-0.025*cos(pi/(ts_1*100).*i);
             0.075+0.025*sin(pi/(ts_1*100).*i);
             0.105];
    Tsp1 = T0c*[Rct,Straj_sp1;0 0 0 1];
    Tsp2 = T0c*[Rct,Straj_sp2;0 0 0 1];
    realTsp1(:,:,i) = Tsp1;
    realTsp2(:,:,i) = Tsp2;
end
realTp12 = use_ctraj([Straj_p1,Straj_p2],Rct,T0c,ts_2*100);
realT = cat(3,realTsp1,realTp12,realTsp2);
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

%%
%plot_robot_jspace(m,Puma,qlist)
%figure
%plot(tlist,qlist(:,1),'*')
%hold on
%plot(tlist,qlist(:,2),'*')
%plot(tlist,qlist(:,3),'*')
%plot(tlist,qlist(:,4),'*')
%plot(tlist,qlist(:,5),'*')
%plot(tlist,qlist(:,6),'*')

