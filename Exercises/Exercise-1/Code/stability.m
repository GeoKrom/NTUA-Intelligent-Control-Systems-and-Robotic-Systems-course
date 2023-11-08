%% Stability Analysis Exable
% Assignment 1 - Exercise 3
%       Name: George Krommydas
%       A.M.: 02121208

clc;
clear;
close all;
syms z y k l


%% Dynamical System
x = 0.5:0.000001:2.5;
x_dot = -(x-1).*((x-2).^2);

% Circle Parameters
R = 0.5;
r = 0.3;
z0 = 0;
y0 = 2;

% Plot
figure(1);
plot(x, x_dot,'r-');
hold on;
plot(x, zeros(size(x)),'b--');
fimplicit((y-y0).^2 + (z-z0).^2 - R^2,'k-');
fimplicit((k-y0).^2 + (l-z0).^2 - r^2,'k-');
plot(1.80279,-0.0312219, 'kx','LineWidth',2);
plot(1,0,'ko','LineWidth',1);
plot(2,0,'ko','LineWidth',1);
axis equal;
grid minor;
xlabel("$x$","Interpreter","latex","FontSize",16);
ylabel("$\dot{x}$","Interpreter","latex","FontSize",16)