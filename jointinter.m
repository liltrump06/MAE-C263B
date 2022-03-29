function [qlist,qdlist,qddlist] = jointinter(a,t_joint,t_mid,v_mid,Traj,mneed,utneed)
%% Parabolic blend - joint interpolation
% Input - see timeprepare.m and parameter_prepare.m
% Output
% qlist - every point in the trajectory in joint space, corresponded to
% each point in samplist.
% qdlist - the joint velocity of every point in the trajectory, corresponded to
% each point in samplist.
% qddlist - the joint acceleration of every point in the trajectory, corresponded to
% each point in samplist.
    [j_num,proc_num] = size(t_joint);
    for j = 1:j_num
        for l = 1:size(mneed(:,1))
            count = mneed(l,2*j-1);
            jom = mneed(l,2*j);
            if l == 1 && jom == 1 
                s1 = sign(v_mid(j,1)-0);
                Q(l) = Traj(j,1) + 0.5*(v_mid(j,1)/t_joint(j,1))*utneed(j,l)^2;
                V(l) = a(j)*utneed(j,l)*s1;
                A(l) = v_mid(j,1)/t_joint(j,1);
            elseif  count == proc_num
                tinb = utneed(j,l) - (0.5*t_joint(j,proc_num-1) + t_mid(j,proc_num-1));
                Qinb = Traj(j,proc_num-1) + v_mid(j,proc_num-1)*(t_joint(j,proc_num-1)/2+t_mid(j,proc_num-1));
                Q(l) = Qinb + v_mid(j,proc_num-1)*utneed(j,l) - 0.5*v_mid(j,proc_num-1)*utneed(j,l)^2/t_joint(j,proc_num);
                amid = sign(0-v_mid(j,proc_num-1))*a(j);
                V(l) = v_mid(j,proc_num-1)-v_mid(j,proc_num-1)*utneed(j,l)/t_joint(j,proc_num);
                A(l) = v_mid(j,proc_num-1)/t_joint(j,proc_num);
            elseif jom == 2
                Q(l) = Traj(j,count) + (t_joint(j,count)/2)*v_mid(j,count)+v_mid(j,count)*(utneed(j,l));
                V(l) = v_mid(j,count);
                A(l) = 0;
            else 
                amid = sign(v_mid(j,count)-v_mid(j,count-1))*a(j);
                tinb = utneed(j,l) + (0.5*t_joint(j,count) + t_mid(j,count));
                Q(l) = Traj(j,count) + v_mid(j,count-1)*tinb + 0.5*amid*utneed(j,l)^2;
                V(l) = v_mid(j,count-1)+amid*utneed(j,l);
                A(l) = amid;
            end
        end
        qlist(:,j) = Q;
        qdlist(:,j) = V;
        qddlist(:,j) = A;
    end
%%
        