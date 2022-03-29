clc;clear;close all
tic
syms a2 a3 d2 d3 d4 
a2 = 0.4318;
a3 = -0.0203;
d3 = -0.0934;
d4 = 0.4331;
% Define the robot
P6G = [0 0 0.05625];
PGT = [-0.1 0 0.08];
P6T = P6G+PGT;
tool = transl(P6T);
L(1) = Link('revolute','d',0,'a',0,'alpha',0,'modified','qlim',[-pi,pi]);
L(2) = Link('revolute','d',0,'a',0,'alpha',-pi/2,'modified','qlim',[-3/4*pi,3/4*pi]);
L(3) = Link('revolute','d',d3,'a',a2,'alpha',0,'modified','qlim',[-3/4*pi,3/4*pi]);
L(4) = Link('revolute','d',d4,'a',a3,'alpha',-pi/2,'modified','qlim',[-pi,pi]);
L(5) = Link('revolute','d',0,'a',0,'alpha',pi/2,'modified','qlim',[-3/4*pi,3/4*pi]);
L(6) = Link('revolute','d',0,'a',0,'alpha',-pi/2,'modified','qlim',[-3/4*pi,3/4*pi]);
Puma = SerialLink(L,'tool',tool,'name','Puma');
Puma.plot([0 0 0 0 0 0])
syms t1 t2 t3 t4 t5 t6 real
q = [t1 t2 t3 t4 t5 t6];
% q = [0 0 0 0 0 0];
T01 = Puma.A([1],q).T;
T12 = Puma.A([2],q).T;
T23 = Puma.A([3],q).T;
T34 = Puma.A([4],q).T;
T45 = Puma.A([5],q).T;
T56 = Puma.A([6],q).T;
T06 = T01*T12*T23*T34*T45*T56;
T0T = T06*tool;
TT6 = inv(SE3(tool));
T03 = T01*T12*T23;




%% Trajectory
la = 0.02;lb=0.02 ;lc = 0.02;%to avoid 
R0c = [cos(la) -sin(la) 0;sin(la) cos(la) 0; 0 0 1]*[cos(lb) 0 sin(lb);0 1 0; -sin(lb) 0 cos(lb)]*[1 0 0;0 cos(lc) -sin(lc);0 sin(lc) cos(lc)];
p0c = [0.4;0.1;0];
T0c = [R0c,p0c;
       0 0 0 1];
figure
format short
[tlisttr1,qlisttr1] = Config_S_traj(Puma,T0c);
[seulneed,sanglerr,serrlist,stlist,sqlist,sqdlist] = S_traj(Puma,T0c,qlisttr1(1000,:));
[tlisttr2,qlisttr2] =S2B_traj(Puma,T0c,sqlist(2000,:));
[beulneed,banglerr,berrlist,btlist,bqlist,bqdlist] = B_traj(Puma,T0c,qlisttr2(1000,:));
[tlisttr3,qlisttr3] =B2H_traj(Puma,T0c,bqlist(2000,:));
[heulneed,hanglerr,herrlist,htlist,hqlist,hqdlist] = H_traj(Puma,T0c,qlisttr3(1000,:));
[tlisttr4,qlisttr4] =H_Config_traj(Puma,T0c,hqlist(2000,:));
%%
qlist = [qlisttr1;sqlist;qlisttr2;bqlist;qlisttr3;hqlist;qlisttr4];
qdlist = [sqdlist;bqdlist;hqdlist];
tlist = [tlisttr1;stlist+10;tlisttr2+30;btlist+40;tlisttr3+60;htlist+70;tlisttr4+90];
m=50;
%%
% plot
plot_robot_jspace(m,Puma,qlist)
%%
figure
subplot(6,1,1)
eulneed = [seulneed;beulneed;heulneed];
plot(eulneed(:,1))
hold on
plot(eulneed(:,2))
plot(eulneed(:,3))
title('tip euler angle vs time')
legend('alpha','beta','gamma')
%%
clist = [];
for lp = 1:size(qlist,1)
    T = Puma.fkine(qlist(lp,:)).T;
    clist = [clist,T(1:3,4)];
end

subplot(6,1,2)
plot(clist(1,:),'*')
hold on
plot(clist(2,:),'*')
plot(clist(3,:),'*')
title('tip position vs time')
legend('x','y','z')
%%
subplot(6,1,3)
plot(serrlist,'*')
hold on
plot(berrlist,'*')
plot(herrlist,'*')
title('tip position error vs time')
legend('x','y','z')
%% 
subplot(6,1,4)
anglerr = [sanglerr;banglerr;hanglerr];
plot(anglerr(:,1))
hold on
plot(anglerr(:,2))
plot(anglerr(:,3))
title('tip orientation error vs time')
legend('alpha','beta','gamma')
%%
subplot(6,1,5)
plot(tlist,qlist(:,1),'*')
hold on
plot(tlist,qlist(:,2),'*')
plot(tlist,qlist(:,3),'*')
plot(tlist,qlist(:,4),'*')
plot(tlist,qlist(:,5),'*')
plot(tlist,qlist(:,6),'*')
title('six angles value vs time')
legend('t1','t2','t3','t4','t5','t6')
%%
subplot(6,1,6)
plot(tlist(1:6000,:),qdlist(:,1),'*')
hold on
plot(tlist(1:6000,:),qdlist(:,2),'*')
plot(tlist(1:6000,:),qdlist(:,3),'*')
plot(tlist(1:6000,:),qdlist(:,4),'*')
plot(tlist(1:6000,:),qdlist(:,5),'*')
plot(tlist(1:6000,:),qdlist(:,6),'*')
title('six angles velocity value vs time')
legend('t1','t2','t3','t4','t5','t6')
