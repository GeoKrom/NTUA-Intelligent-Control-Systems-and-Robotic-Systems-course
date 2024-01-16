%% Two DoF Robotic Manipulator Backstepping and Adaptive Backstepping Control
%       Assignment 4
%       Name: George Krommydas
%       A.M.: 02121208

clear;
clc;


global m1
global l1
global Iz1
global k1
global m2
global l2
global Iz2
global k2
global g
global parvar

%% Parameter Setting

m1 = 3.2;                   % kg
l1 = 0.5;                   % m
Iz1 = 0.96;                 % kg m^2
k1 = 1;                     % kg m^2/sec
m2 = 2.0;                   % kg
l2 = 0.4;                   % m
Iz2 = 0.81;                 % kg m^2
k2 = 1;                     % kg m^2/sec
g = 9.81;                   % m/s^2

tstart = 0;                 % sec
tend = 60;                  % sec
tspan = [tstart, tend];
q1_0 = 80*pi/180;
q2_0 = 130*pi/180;
x0 = [q1_0;q2_0;0;0];
parvar = [Iz1+m1*(l1^2/4)+m2*l1^2;
              Iz2 + m2*(l2^2/4);
              0.5*m2*l1*l2;
              k1;
              k2;
              0.5*m1*g*l1+m2*g*l1;
              0.5*m2*g*l2];
% Simulation

[t,x] = ode45(@BacksteppingControl, tspan, x0);

q1_d = 90*pi/180 + (30*pi/180)*cos(t);
q1_dot_d = -(30*pi/180)*sin(t);
q1_ddot_d = -(30*pi/180)*cos(t);
q2_d = 90*pi/180 - (30*pi/180)*sin(t);
q2_dot_d = -(30*pi/180)*cos(t);
q2_ddot_d = (30*pi/180)*sin(t);

% Plots
figure(1);
clf;

subplot(2,1,1);
plot(t, x(:,1),'r-');
hold on;
plot(t, q1_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$q_1 [rad]$','Interpreter','latex','FontSize',14);
grid on;
legend('$q_1$', '$q_{1_d}$','Interpreter','latex',Location='southeast');

subplot(2,1,2);
plot(t, x(:,3),'r-');
hold on;
plot(t, q1_dot_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{q}_1 [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\dot{q}_1$', '$\dot{q}_{1_d}$','Interpreter','latex',Location='southeast');


figure(2);
clf;

subplot(2,1,1);
plot(t, x(:,2),'r-');
hold on;
plot(t, q2_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$q_2 [rad]$','Interpreter','latex','FontSize',14);
grid on;
legend('$q_2$', '$q_{2_d}$','Interpreter','latex',Location='southeast');

subplot(2,1,2);
plot(t, x(:,4),'r-');
hold on;
plot(t, q2_dot_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{q}_2 [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\dot{q}_2$', '$\dot{q}_{2_d}$','Interpreter','latex',Location='southeast');

figure(3);
clf;

subplot(4,1,1);
plot(t, x(:,1) - q1_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{q_1} [rad]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,2);
plot(t, x(:,3) - q1_dot_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{e}_{q_1} [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,3);
plot(t, x(:,2) - q2_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{q_2} [rad]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,4);
plot(t, x(:,4) - q2_dot_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{e}_{q_2} [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;

% Simulation Adaptive Backstepping
x01 = [x0; zeros(size(parvar))];
[t1, x1] = ode45(@AdaptiveBacksteppingControl, tspan, x01);

q1_d = 90*pi/180 + (30*pi/180)*cos(t1);
q1_dot_d = -(30*pi/180)*sin(t1);
q1_ddot_d = -(30*pi/180)*cos(t1);
q2_d = 90*pi/180 - (30*pi/180)*sin(t1);
q2_dot_d = -(30*pi/180)*cos(t1);
q2_ddot_d = (30*pi/180)*sin(t1);

% Plots
figure(4);
clf;

subplot(2,1,1);
plot(t1, x1(:,1),'r-');
hold on;
plot(t1, q1_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$q_1 [rad]$','Interpreter','latex','FontSize',14);
grid on;
legend('$q_1$', '$q_{1_d}$','Interpreter','latex',Location='southeast');

subplot(2,1,2);
plot(t1, x1(:,3),'r-');
hold on;
plot(t1, q1_dot_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{q}_1 [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\dot{q}_1$', '$\dot{q}_{1_d}$','Interpreter','latex',Location='southeast');


figure(5);
clf;

subplot(2,1,1);
plot(t1, x1(:,2),'r-');
hold on;
plot(t1, q2_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$q_2 [rad]$','Interpreter','latex','FontSize',14);
grid on;
legend('$q_2$', '$q_{2_d}$','Interpreter','latex',Location='southeast');

subplot(2,1,2);
plot(t1, x1(:,4),'r-');
hold on;
plot(t1, q2_dot_d, 'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{q}_2 [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\dot{q}_2$', '$\dot{q}_{2_d}$','Interpreter','latex',Location='southeast');

figure(6);
clf;

subplot(4,1,1);
plot(t1, x1(:,1) - q1_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{q_1} [rad]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,2);
plot(t1, x1(:,3) - q1_dot_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{e}_{q_1} [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,3);
plot(t1, x1(:,2) - q2_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{q_2} [rad]$','Interpreter','latex','FontSize',14);
grid on;

subplot(4,1,4);
plot(t1, x1(:,4) - q2_dot_d,'r-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\dot{e}_{q_2} [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;

figure(7);
clf;

subplot(4,1,1);
plot(t1, x1(:,5),'r-');
hold on;
plot(t1, parvar(1).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_1$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_1$', '$\pi_{1_d}$','Interpreter','latex',Location='southeast');

subplot(4,1,2);
plot(t1, x1(:,6),'r-');
hold on;
plot(t1, parvar(2).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_2$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_2$', '$\pi_{2_d}$','Interpreter','latex',Location='southeast');

subplot(4,1,3);
plot(t1, x1(:,7),'r-');
hold on;
plot(t1, parvar(3).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_3$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_3$', '$\pi_{3_d}$','Interpreter','latex',Location='southeast');

subplot(4,1,4);
plot(t1, x1(:,8),'r-');
hold on;
plot(t1, parvar(4).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_4$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_4$', '$\pi_{4_d}$','Interpreter','latex',Location='southeast');

figure(8);
clf;

subplot(3,1,1);
plot(t1, x1(:,9),'r-');
hold on;
plot(t1, parvar(5).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_5$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_5$', '$\pi_{5_d}$','Interpreter','latex',Location='southeast');

subplot(3,1,2);
plot(t1, x1(:,10),'r-');
hold on;
plot(t1, parvar(6).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_6$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_6$', '$\pi_{6_d}$','Interpreter','latex',Location='southeast');

subplot(3,1,3);
plot(t1, x1(:,11),'r-');
hold on;
plot(t1, parvar(7).*ones(size(t1)),'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\pi_7$','Interpreter','latex','FontSize',14);
grid on;
legend('$\pi_7$', '$\pi_{7_d}$','Interpreter','latex',Location='southeast');


















