 clc; close all; clear;

syms q1 q2 q3 real;
syms dq1 dq2 dq3 real;
% syms ddq1 ddq2 ddq3 real;
% syms t t_end real;

q0 = [0; 0; pi/2];
q0_1 = q0(1);
q0_2 = q0(2);
q0_3 = q0(3);
dq0 = [0; 0; 0];
dq0_1 = dq0(1);
dq0_2 = dq0(2);
dq0_3 = dq0(3);

T=  5;
Ts=0.001;

syms L dc1 dc2 dc3 m1 m2 m3 I1 I2 I3 g0 real;

[M, n] = dynamic_model_3R([q1;q2;q3], [dq1;dq2;dq3], [L dc1 dc2 dc3 m1 m2 m3 I1 I2 I3 g0],0);

simulation = sim('simulink_model');

plot(simulation.q1.time, simulation.q1.signals.values);
hold on;
plot(simulation.q2.time, simulation.q2.signals.values);
hold on;
plot(simulation.q3.time, simulation.q3.signals.values);
legend('q1', 'q2', 'q3');

figure;
plot(simulation.tau1.time, simulation.tau1.signals.values);
hold on;
plot(simulation.tau2.time, simulation.tau2.signals.values);
hold on;
plot(simulation.tau3.time, simulation.tau3.signals.values);
legend('tau1', 'tau2', 'tau3');

