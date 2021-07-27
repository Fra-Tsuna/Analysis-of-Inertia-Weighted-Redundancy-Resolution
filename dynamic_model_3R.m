function [M,n] = dynamic_model_3R(q,dq,params,plane)
% plane = 1 -> horizontal, no gravity

q1 = q(1);
q2 = q(2);
q3 = q(3);
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
% 
% L = 1;
% dc1 = 1/2 * L;
% dc2 = 1/2 * L;
% dc3 = 1/2 * L;
% m1 = 10;
% m2 = 10;
% m3 = 10;
% I1 = 1/12 * m1 * L^2;
% I2 = 1/12 * m2 * L^2;
% I3 = 1/12 * m3 * L^2;

L = params(1);
dc1 = params(2);
dc2 = params(3);
dc3 = params(4);
m1 = params(5);
m2 = params(6);
m3 = params(7);
I1 = params(8);
I2 = params(9);
I3 = params(10);
g0 = params(11);

pc1 = [dc1 * cos(q1); dc1* sin(q1)];
vc1 = [diff(pc1, q1) * dq1];

T1 = 0.5 * (m1 * vc1'*vc1) + 0.5 * (I1 * dq1^2);

pc2 = [L * cos(q1) + dc2 * cos(q1 + q2) ; L * sin(q1) + dc2 * sin(q1+q2)];
vc2 = [diff(pc2,q1) * dq1 + diff(pc2, q2) * dq2];

T2 = 0.5 * (m2 * vc2' * vc2) + 0.5 * I2 * (dq1+ dq2)^2;

pc3 = [L*cos(q1) + L * cos(q1+q2) + dc3 * cos(q1+q2+q3); L*sin(q1) + L * sin(q1+q2) + dc3* sin(q1+q2+q3)];
vc3 = [diff(pc3,q1) * dq1 + diff(pc3, q2) * dq2 + diff(pc3,q3) * dq3];


T3 = 0.5 * (m3 * vc3' * vc3) + 0.5 * I3 * (dq1+dq2+dq3)^2;

T  = T1 + T2 + T3;
T = collect(T, dq1^2);
T = collect(T, dq2^2);
T = collect(T, dq3^2);


M(1,1) = diff(diff(T,dq1),dq1);
M(2,2) = diff(diff(T,dq2),dq2);
M(3,3) = diff(diff(T,dq3),dq3);
M(1,2) = diff(diff(T,dq1),dq2);
M(1,3) = diff(diff(T,dq1),dq3);
M(2,3) = diff(diff(T,dq2),dq3);
M(2,1) = M(1,2);
M(3,1) = M(1,3);
M(3,2) = M(2,3);

M = simplify(M);

a1 = I1 + I2 + I3 + L^2*m2 + 2*L^2*m3 + dc1^2*m1 + dc2^2*m2 + dc3^2*m3;
a2 = L^2*m3 + L*dc2*m2;
a3 = L*dc3*m3;
a4 = I2 + I3 + L^2*m3 + dc2^2*m2 + dc3^2*m3;
a5 = I3 + dc3^2*m3;

M = subs(M, I1 + I2 + I3 + L^2*m2 + 2*L^2*m3 + dc1^2*m1 + dc2^2*m2 + dc3^2*m3, a1);
M = simplify(M);
M = collect(M,cos(q2));
M = simplify(M);
M = subs(M, L*dc3*m3, a3);
M = simplify(M);
M = subs(M, I2 + I3 + L^2*m3 + dc2^2*m2 + dc3^2*m3, a4);
M = subs(M, L^2*m3 + L*dc2*m2, a2);
M = subs(M, I3 + dc3^2*m3, a5);

% Coriolis
M1 = M(:,1);
M2 = M(:,2);
M3 = M(:,3);
C1 = 0.5*(jacobian(M1,q) + jacobian(M1,q)' - diff(M,q1));
C2 = 0.5*(jacobian(M2,q) + jacobian(M2,q)' - diff(M,q2));
C3 = 0.5* (jacobian(M3,q) + jacobian(M3,q)' - diff(M, q3));

c1 = dq' * C1 * dq;
c2 = dq' * C2 * dq;
c3 = dq' * C3 * dq;

c = [c1;c2;c3];
c = simplify(c);

%ENERGIA POTENZIALE

U1 = m1 * g0 * (dc1 * sin(q1));
U2 = m2 * g0 * (L * sin(q1) + dc2 * sin(q1+q2));
U3 = m3* g0* (L * sin(q1) + L * sin(q1+q2) + dc3 * sin(q1+q2+q3));

U = U1+U2+U3;
g = jacobian(U,q)';
g = simplify(g);


M = subs(M);
c = subs(c);
g = subs(g);

if (plane == 1)
    n = c;
    else
    n = c + g;
end
end