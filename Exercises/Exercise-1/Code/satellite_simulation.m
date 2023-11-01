%% Orbit simulation of a satellite with its linear and nonlinear system
% Assignment 1
% Exercise 1
%       Name: George Krommydas
%       A.M.: 02121208

clear;
clc;

%% Parameters
global fe
global fa
global k


fa = 0;                     % N
fe = -1;                     % N
r0 = 4.218709065*1e7;       % m
w0 = 7.29219108*1e-5;       % rad/sec
k = w0^2*r0^3;              % N
t_span = [0 285120];        % sec
%t_span = [0 11880];        % sec
%% Nonlinear System Solving
x0 = [r0; 0; 0; w0];

[t,x] = ode45(@equations, t_span, x0);

%% Linear System Solving

A = [0      1       0  0;
     3*w0^2 0       0  2*r0*w0;
     0      0       0  1;
     0     -2*w0/r0 0  0];
B = [0  0;
     1  0;
     0  0;
     0  1/r0];
C = eye(4)';
D = 0;
u = [fa; fe].*ones(size(t_span,2));
z0 = [0; 0; 0; 0];
%z0 = [0; 0; 0; 0];
sys = ss(A,B,C,D);
z = lsim(sys,u,t_span,z0);

%% Nonlinear Model Simulation

figure(1);
clf;
subplot(2,2,1);
plot(t, x(:,1),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\varrho  [m]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,3);
plot(t, x(:,2),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\dot{\varrho}  [m/sec]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,2);
plot(t, x(:,3),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\vartheta  [rad]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,4);
plot(t, x(:,4),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\omega  [rad/sec]$', "Interpreter", "latex","FontSize",14);
grid minor;

%% Linear Model Simulation 

figure(2);
clf;
subplot(2,2,1);
plot(t_span, z(:,1),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\varrho  [m]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,3);
plot(t_span, z(:,2),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\dot{\varrho}  [m/sec]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,2);
plot(t_span, z(:,3),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\vartheta  [rad]$', "Interpreter", "latex","FontSize",14);
grid minor;

subplot(2,2,4);
plot(t_span, z(:,4),'b-');
xlabel('t [sec]', "Interpreter", "latex","FontSize",14);
ylabel('$\omega  [rad/sec]$', "Interpreter", "latex","FontSize",14);
grid minor;