function J = Jacobian_cal(PUMA,q)

%q = [0 0 pi/6 pi/2 0.3 0]
T6T = SE3(PUMA.tool);
T01 = PUMA.A([1],q);
T02 = PUMA.A([1 2],q);
T03 = PUMA.A([1 2 3],q);
T04 = PUMA.A([1 2 3 4],q);
T05 = PUMA.A([1 2 3 4 5],q);
T06 = PUMA.A([1 2 3 4 5 6],q);
T0T = T06*T6T;

[R01,P01] = tr2rt(T01);
[R02,P02] = tr2rt(T02);
[R03,P03] = tr2rt(T03);
[R04,P04] = tr2rt(T04);
[R05,P05] = tr2rt(T05);
[R06,P06] = tr2rt(T06);
[R0T,P0T] = tr2rt(T0T);

J = sym(zeros(6,6));
J(:,1) = [cross(R01(:, 3), (P0T - P01));R01(:,3)];
J(:,2) = [cross(R02(:, 3), (P0T - P02));R02(:,3)];
J(:,3) = [cross(R03(:, 3), (P0T - P03));R03(:,3)];
J(:,4) = [cross(R04(:, 3), (P0T - P04));R04(:,3)];
J(:,5) = [cross(R05(:, 3), (P0T - P05));R05(:,3)];
J(:,6) = [cross(R06(:, 3), (P0T - P06));R06(:,3)];
J = J;