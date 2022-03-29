function Tlist = use_ctraj(traj,Rct2,T0c,points)

%use ctraj
Tcpsp1 = [Rct2,traj(:,1);
       0 0 0 1];
T0sp1=T0c*Tcpsp1;
Tcpsp2 =[Rct2,traj(:,2);
           0 0 0 1];
T0sp2=T0c*Tcpsp2;
Tlist = ctraj(T0sp1,T0sp2,points);