function [alist,tmlist,t_joint,t_mid,v_mid] = parameter_prepare(tm,a,Traj)
%% Parabolic blend - parameter prepare
% Input
% n(joints), m(points)
% tm - time from one point to another, it can be a list or a scaler.(1 x
% m-1) or 1
% a - acceleration list of all joints.
% Traj - all points of the trajectory in joint space. (n x m)
% Output
% alist - acceleration list of all joints (n x 1)
% tmlist - whole time list of each mid process (1 x m-1)
% t_joint - time length when accelerating (n x m)
% t_mid - time length that joint with static velocity (n x m-1)
% v_mid - velocity at t_mid (n x m-1)
%%

[lm,ln] = size(Traj);
if size(tm,2)<2
    tmlist = ones(1,lm-1)*tm;
else 
    tmlist = tm;
end
if size(a,2)<2
    alist = ones(1,lm-1)*a;
else 
    alist = a;
end
op = 1:ln;
    for j = op
        a = sign(Traj(2,j) - Traj(1,j))*alist(j);
        tk(1) = tmlist(1) - sqrt(tmlist(1)^2 - 2*(Traj(2,j)-Traj(1,j))/a);
        vjk(1) = (Traj(2,j) - Traj(1,j))/(tmlist(1) - 0.5 * tk(1));

        for l = 2 : lm-1
            vjk(l) = (Traj(l+1,j) - Traj(l,j))/tmlist(l);
            a = sign(vjk(l) - vjk(l-1))*alist(j);
            tk(l) = (vjk(l) - vjk(l-1))/ a;
            tjk(l-1) = tmlist(l) - 0.5*tk(l-1) - 0.5*tk(l);
        end

        alast = sign(Traj(lm,j) - Traj(lm-1,j))*alist(j);
        tk(lm)= tmlist(lm-1) - sqrt(tmlist(lm-1)^2 - 2*(Traj(lm,j)-Traj(lm-1,j))/alast);

        tjk(1) = tmlist(1)-tk(1)-0.5*tk(2);
        vjk(lm-1) = (Traj(lm,j) - Traj(lm-1,j))/(tmlist(lm-1) - 0.5 * tk(lm));
        a = sign(vjk(lm-1) - vjk(lm-2))*alist(j);
        tk(lm-1) = (vjk(lm-1) - vjk(lm-2))/ a;
        tjk(lm-2) = tmlist(lm-2)-0.5*tk(lm-2)-0.5*tk(lm-1);
        tjk(lm-1) = tmlist(lm-1)-0.5*tk(lm-1)-tk(lm);
        
        t_joint(j,:) = tk;
        t_mid(j,:) = tjk;
        v_mid(j,:) = vjk;
    end

end

