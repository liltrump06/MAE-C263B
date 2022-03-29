function [samplist,mneed,utneed] = timeprepare(t_joint,t_mid,rate)
%% parabolic blend - time prepare
% Aim 
% Since data of joints can only be observed at discrete intervals,
% reset the time interval according to sampling rate.
% Input
% rate - sampling rate, scaler
% t_joint - see parameter_prepare.m
% t_mid - see parameter_prepare.m
% Output
% samplist - time points given by sampling rate and the whole time (t_joint
% + t_mid) ((t_joint/(1,:)+t_mid/(1,:))/rate x 1)
% mneed - index to identify every point in samplist is corresponded to which time intervals.
% the second column tells the point is in t_joint or t_mid, value 1 means
% t_joint, value 2 means t_mid. the first column means its index in t_mid
% or t_joint. ((t_joint/(1,:)+t_mid/(1,:))/rate x 2)
% utneed - time point at each interval ((t_joint/(1,:)+t_mid/(1,:))/rate x
% 1)
%%
mneed = [];
    tw = sum(t_mid(1,:))+sum(t_joint(1,:));
    for j = 1:6
            mlist = [];
            utlist = [];
            samplist = [];
        for t = 0:rate:tw-rate
            tcl=0;
            for m = 1:size(t_joint,2)
                
                tcl = tcl + t_joint(j,m);
                if t < tcl
                    samplist = [samplist;t];
                    mlist = [mlist;[m,1]];
                    utlist = [utlist;t-(tcl-t_joint(j,m))];
                    break
                end
                    if m==size(t_joint,2)
                        break
                    else
                        tcl = t_mid(j,m)+tcl;
                    end
                if t < tcl
                    samplist = [samplist;t];
                    mlist = [mlist;[m,2]];
                    utlist = [utlist;t-(tcl-t_mid(j,m))];
                    break
                end
            end
        end
        mneed  =[mneed,mlist];
        utneed(j,:) = utlist;
    end
