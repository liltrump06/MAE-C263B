function [qlist,qdlist,qddlist] = make_curve(points1, points2,v_via,thsplist1,t)
qlist = [];
qdlist = [];
qddlist = [];
if points1 == 2
    [q,qd,qdd] = jtraj(thsplist1(1,:),thsplist1(2,:),linspace(0,t,points2),zeros(1,6),zeros(1,6));
    qlist = [qlist;q];
    qdlist = [qdlist;qd];
    qddlist = [qddlist;qdd];
else
    for m = 1:points1
        if m == points1 - 1
            [q,qd,qdd] = jtraj(thsplist1(m,:),thsplist1(m+1,:),linspace(0,t/(points1-1),points2),v_via*ones(1,6),zeros(1,6));
            qlist = [qlist;q(2:points2,:)];
            qdlist = [qdlist;qd(2:points2,:)];
            qddlist = [qddlist;qdd(2:points2,:)];
            break
        elseif m == 1
            [q,qd,qdd] = jtraj(thsplist1(m,:),thsplist1(m+1,:),linspace(0,t/(points1-1),points2),zeros(1,6),v_via*ones(1,6));
            qlist = [qlist;q];
            qdlist = [qdlist;qd];
            qddlist = [qddlist;qdd];
            continue
        else
            [q,qd,qdd] = jtraj(thsplist1(m,:),thsplist1(m+1,:),linspace(0,t/(points1-1),points2),v_via*ones(1,6),v_via*ones(1,6));
            qlist = [qlist;q(2:points2,:)];
            qdlist = [qdlist;qd(2:points2,:)];
            qddlist = [qddlist;qdd(2:points2,:)];
        end    
    end
end

