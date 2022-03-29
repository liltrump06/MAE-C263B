function thsplist = c2j_curve_c(robot,Rct2,T0c,traj,points,q0)
thsplist=[];
thsp = zeros(1,6);
%use ctraj
Tcpsp1 = [Rct2,traj(:,1);
       0 0 0 1];
T0sp1=T0c*Tcpsp1;
Tcpsp2 =[Rct2,traj(:,2);
           0 0 0 1];
T0sp2=T0c*Tcpsp2;
Tlist = ctraj(T0sp1,T0sp2,points);
for i = 1:size(Tlist,3)
   if i == 1
    thsp = robot.ikine(Tlist(:,:,i),'q0',q0);
    thsplist = [thsplist ;thsp];
   else
    thsp = robot.ikine(Tlist(:,:,i),'q0',thsp);
    thsplist = [thsplist ;thsp];
   end
end