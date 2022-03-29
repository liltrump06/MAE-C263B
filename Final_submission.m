%% 263B Final
%% Part1 Kinematic analysis
%% 1.Forward Kinematics
%First,define the robot by using modified DH parameters.
%Plot the initial configuration.
clc;clear;close all
tic
pi = sym(pi)
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
L(1) = Link('revolute','d',0,'a',0,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',0,'alpha',-pi/2,'modified');
L(3) = Link('revolute','d',d3,'a',a2,'alpha',0,'modified');
L(4) = Link('revolute','d',d4,'a',a3,'alpha',-pi/2,'modified');
L(5) = Link('revolute','d',0,'a',0,'alpha',pi/2,'modified');
L(6) = Link('revolute','d',0,'a',0,'alpha',-pi/2,'modified');
Puma = SerialLink(L,'tool',tool,'name','Puma');
% Then using the symbolic toolbox to form the FK equation.
syms t1 t2 t3 t4 t5 t6 real
q = [t1 t2 t3 t4 t5 t6];
% q = [0 0 0 0 0 0];
T01 = Puma.A([1],q).T
T12 = Puma.A([2],q).T;
T12(2,1) = 0;
T12(2,2) = 0;
T12(3,3) = 0;
T12(3,4) = 0;
T12
T23 = Puma.A([3],q).T
T34 = Puma.A([4],q).T;
T34(2,1) = 0;
T34(2,2) = 0;
T34(3,3) = 0;
T34(3,4) = 0;
T34
T45 = Puma.A([5],q).T;
T45(2,1) = 0;
T45(2,2) = 0;
T45(3,3) = 0;
T45
T56 = Puma.A([6],q).T;
T56(2,1) = 0;
T56(2,2) = 0;
T56(3,3) = 0;
T56
T6T = [1 0 0 -0.1;0 1 0 0;0 0 1 0.1363;0 0 0 1];
T06 = simplify(T01*T12*T23*T34*T45*T56);
T0T = simplify(T06*T6T);
TT6 = inv(T6T)
T03 = simplify(T01*T12*T23)

%% 2.inverse kinematics
%Using algebraic solution to calculate inverse.
%This function means the method to calculate inverse, and it is posted at the last of this file.


%% 3. Jacobian matrix.
%Using explicit method to calculate Jacobian.
T01 = Puma.A([1],q);
T02 = Puma.A([1 2],q);
T03 = Puma.A([1 2 3],q);
T04 = Puma.A([1 2 3 4],q);
T05 = Puma.A([1 2 3 4 5],q);
T06 = Puma.A([1 2 3 4 5 6],q);
T0T = T06*SE3(T6T);

[R01,P01] = tr2rt(T01);
[R12,P12] = tr2rt(T12);
[R23,P23] = tr2rt(T23);
[R34,P34] = tr2rt(T34);
[R45,P45] = tr2rt(T45);
[R56,P56] = tr2rt(T56);

[R02,P02] = tr2rt(T02);
[R03,P03] = tr2rt(T03);
[R04,P04] = tr2rt(T04);
[R05,P05] = tr2rt(T05);
[R06,P06] = tr2rt(T06);
[R0T,P0T] = tr2rt(T0T);
%with tool
J = sym(zeros(6,6));
J(:,1) = [cross(R01(:, 3), (P0T - P01));R01(:,3)];
J(:,2) = [cross(R02(:, 3), (P0T - P02));R02(:,3)];
J(:,3) = [cross(R03(:, 3), (P0T - P03));R03(:,3)];
J(:,4) = [cross(R04(:, 3), (P0T - P04));R04(:,3)];
J(:,5) = [cross(R05(:, 3), (P0T - P05));R05(:,3)];
J(:,6) = [cross(R06(:, 3), (P0T - P06));R06(:,3)];
J = simplify(J)
%no tool

J_notool = sym(zeros(6,6));
J_notool(:,1) = [cross(R01(:, 3), (P06 - P01));R01(:,3)];
J_notool(:,2) = [cross(R02(:, 3), (P06 - P02));R02(:,3)];
J_notool(:,3) = [cross(R03(:, 3), (P06 - P03));R03(:,3)];
J_notool(:,4) = [cross(R04(:, 3), (P06 - P04));R04(:,3)];
J_notool(:,5) = [cross(R05(:, 3), (P06 - P05));R05(:,3)];
J_notool(:,6) = [cross(R06(:, 3), (P06 - P06));R06(:,3)];
J_notool = simplify(J_notool)
%%
J3 = J_notool;
eqn = det(J3);
eqnsm = simplify(eqn);
m = solve(eqnsm == 0,t1,t2,t3,t4,t5,t6);
t1m = double(m.t1)
t2m = double(m.t2)
t3m = double(m.t3)
t4m = double(m.t4)
t5m = double(m.t5)
t6m = double(m.t6)
Puma.plot([t1m(1),t2m(1),t3m(1),t4m(1),t5m(1),t6m(1)])
%% 2.Dynamics
%dynamics()
%% Trajectory

%trajectory()
%%

function theta = Puma_inverse(M)
a2 = 0.4318;
a3 = 0.1254;
d3 = 0.1254;
d4 = 0.4318;
[RT,FT] = tr2rt(M);
M = [RT,FT;0 0 0 1];
Px = FT(1);
Py = FT(2);
Pz = FT(3);
r11 = RT(1,1);
r12 = RT(1,2);
r13 = RT(1,3);
r21 = RT(2,1);
r22 = RT(2,2);
r23 = RT(2,3);
r31 = RT(3,1);
r32 = RT(3,2);
r33 = RT(3,3);
sign1 = 1;
sign2 = 1;
K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));
c1 = cos(theta1);
s1 = sin(theta1);
theta3 = (atan2(a3,d4)-atan2(real(K),real(sign2*sqrt(a3^2+d4^2-K^2))));
c3 = cos(theta3);
s3 = sin(theta3);
t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
theta2 = (t23 - theta3);
c2 = cos(theta2);
s2 = sin(theta2);
s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);
theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23 + r33*s23);
c4 = cos(theta4);
s4 = sin (theta4);
s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5 = atan2(s5,c5);
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6 = atan2(s6,c6);
theta = [theta1 theta2 theta3 theta4 theta5 theta6];
end

