function [tlist,qlist,qdlist,qddlist] = pre_jointinter(a,t_joint,t_mid,v_mid,Traj,sup1,sup2)
    
    [j_num,proc_num] = size(t_joint);
    for j = 1:j_num
        Qc = [];
        Vc = [];
        Ac = [];
        Tc = [];
        tfj = linspace(0,t_joint(j,1),sup1);
        tcl = 0;
        for t = 0 : sup1-1
            s1 = sign(v_mid(j,1)-0);
            Q1(t+1) = Traj(j,1) + 0.5*(v_mid(j,1)/t_joint(j,1))*tfj(t+1)^2;
            V1(t+1) = a(j)*tfj(t+1)*s1;
            A1(t+1) = v_mid(j,1)/t_joint(j,1);
            T1(t+1) = tfj(t+1)+tcl;
        end
        tcl = t_joint(j,1)+tcl;

        for i = 1 : proc_num-1
            tfm = linspace(0,t_mid(j,i),sup2);
            tfj = linspace(0,t_joint(j,i+1),sup1);


            for t = 1:sup2-1
                Qa(t) = Traj(j,i) + (t_joint(j,i)/2)*v_mid(j,i)+v_mid(j,i)*(tfm(t+1));
                Va(t) = v_mid(j,i);
                Aa(t) = 0;
                Ta(t) = tfm(t+1)+tcl;
            end
            tcl = t_mid(j,i)+tcl;
            if i == proc_num-1
                Qc = [Qc,Qa];
                Vc = [Vc,Va];
                Ac = [Ac,Aa];
                Tc = [Tc,Ta];
                break
            end
            for t = 1:sup1-1
                amid = sign(v_mid(j,i+1)-v_mid(j,i))*a(j);
                tinb = tfj(t+1) + (0.5*t_joint(j,i) + t_mid(j,i));
                Qb(t) = Traj(j,i) + v_mid(j,i)*tinb + 0.5*amid*tfj(t+1)^2;
                Vb(t) = v_mid(j,i)+amid*tfj(t+1);
                Ab(t) = amid;
                Tb(t) = tfj(t+1)+tcl;
            end
            tcl = t_joint(j,i+1)+tcl;
            Qc = [Qc,Qa,Qb];
            Vc = [Vc,Va,Vb];
            Ac = [Ac,Aa,Ab];
            Tc = [Tc,Ta,Tb];
        end
        
        tfj = linspace(0,t_joint(j,proc_num),sup1);
        for t = 1:sup1-1
            tinb = tfj(t+1) - (0.5*t_joint(j,proc_num-1) + t_mid(j,proc_num-1));
            Qinb = Traj(j,proc_num-1) + v_mid(j,proc_num-1)*(t_joint(j,proc_num-1)/2+t_mid(j,proc_num-1));
            Qlast(t) = Qinb + v_mid(j,proc_num-1)*tfj(t+1) - 0.5*v_mid(j,proc_num-1)*tfj(t+1)^2/t_joint(j,proc_num);
            amid = sign(0-v_mid(j,proc_num-1))*a(j);
            Vlast(t) = v_mid(j,proc_num-1)-v_mid(j,proc_num-1)*tfj(t+1)/t_joint(j,proc_num);
            Alast(t) = v_mid(j,proc_num-1)/t_joint(j,proc_num);
            Tlast(t) = tfj(t+1)+tcl;
        end

        qlist(:,j) = [Q1,Qc,Qlast]';
        qdlist(:,j) = [V1,Vc,Vlast]';
        qddlist(:,j) = [A1,Ac,Alast]';
        tlist(:,j) = [T1,Tc,Tlast];
    end

