
clc;clear;close all
tic
pi = sym(pi);
syms a2 a3 d2 d3 d4 
%a2 = 0.4318;
%a3 = -0.0203;
%d3 = -0.0934;
%d4 = 0.4331;
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
syms t1 t2 t3 t4 t5 t6 real
q = [t1 t2 t3 t4 t5 t6];
T01 = Puma.A([1],q).T;
T12 = Puma.A([2],q).T;
T23 = Puma.A([3],q).T;
T34 = Puma.A([4],q).T;
T45 = Puma.A([5],q).T;
T56 = Puma.A([6],q).T;
T01 = Puma.A([1],q).T;
T02 = Puma.A([1 2],q).T;
T03 = Puma.A([1 2 3],q).T;
T04 = Puma.A([1 2 3 4],q).T;
T05 = Puma.A([1 2 3 4 5],q).T;
T06 = Puma.A([1 2 3 4 5 6],q).T;
T6T = [1 0 0 -0.1;0 1 0 0;0 0 1 0.1363;0 0 0 1];
T06 = T01*T12*T23*T34*T45*T56;
T0T = T06*tool;
TT6 = inv(SE3(tool));
T03 = T01*T12*T23;
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

syms t11 t21 t31 t12 t22 t32 real

m1 = 0; m2 = 17.4; m3 = 6.04;



P1_cm = [0; 0; 0];
P2_cm = [0.068; 0.006; -0.016];
P3_cm = [0; -0.143; 0.014];
g = 9.81;
I_c1 = [0 0 0;
        0 0 0;
        0 0 0.35];
I_c2 = [0.13 0 0;
        0 0.524 0;
        0 0 0.539];
I_c3 = [0.192 0 0;
        0 0.0154 0;
        0 0 0.212];
f44 = zeros(3,1); 
n44 = zeros(3,1); 
w00  = zeros(3,1);
al00 = zeros(3,1);
v00  = zeros(3,1); 
a00 = [0 ; -g; 0];

%outward
w11 = R01*w00 + t11 * R01(:,3);
al11 = R01'* al00 + R01' * cross(w00, t11*R01(:,3)) + t12*R01(:,3);
a11 = R01'* (cross(al00, P01) + cross(w00, cross(w00,P01)) + a00);
ac11 = cross(al11, P1_cm) + cross(w11, cross(w11,P1_cm)) + a11;
F11 = m1*ac11;
N1 = I_c1 * al11 + cross(w11,I_c1*w11);
% i= 1
w22 = R12'*w11 + t21 * R12(:,3);
al22 = R12'* al11 + R12' * cross(w11, t21*R12(:,3)) + t22*R12(:,3);
a22 = R12'* (cross(al11, P12) + cross(w11, cross(w11,P12)) + a11);
a22 = simplify(a22);
ac22 = cross(al22, P2_cm) + cross(w22, cross(w22,P2_cm)) + a22;
ac22 = simplify(ac22);
F22 = m2*ac22;
N2 = I_c2 * al22 + cross(w22,I_c2*w22);
% i= 2
w33 = R23'*w22 + t31 * R23(:,3);
al33 = R23'* al22 + R23' * cross(w22, t31*R23(:,3)) + t32*R23(:,3);
a33 = R23'* (cross(al22, P23) + cross(w22, cross(w22,P23)) + a22);
a33 = simplify(a33);
ac33 = cross(al33, P3_cm) + cross(w33, cross(w33,P3_cm)) + a33;
ac33 = simplify(ac33);
%i = 3
F3 = m3*ac33;
N3 = I_c3 * al33 + cross(w33,I_c3*w33);

%% inward
% i = 3
f33 = R34 * f44 + F3;
f33 = simplify(f33);
n33 = N3 + R34*n44 + cross(P3_cm, F3) + cross(P34, R34 * f44);
n33 = simplify(n33);

Tau3 = simplify(transpose(n33)* R23(:,3))

% i = 2
f22 = R23 * f33 + F22;
f22 = simplify(f22);
n22 = N2 + R23*n33 + cross(P2_cm, F22) + cross(P23, R23 * f33);
n22 = simplify(n22);

Tau2 = simplify(transpose(n22)* R12(:,3))

% i = 1
f11 = R12 * f22 + F11;
f11 = simplify(f11);
n11 = N1 + R12*n22 + cross(P1_cm, F11) + cross(P12, R12 * f22);
n22 = simplify(n22);

Tau1 = simplify(transpose(n11)* R01(:,3))
Tau = [Tau1;Tau2;Tau3]