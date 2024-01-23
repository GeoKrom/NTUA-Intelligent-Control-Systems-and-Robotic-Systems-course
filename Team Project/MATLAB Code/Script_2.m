%% Simulation Script 2: Closed Loop System Static Feedback

clc
clear
close all

%% Parameter Init

%Constant Parameters
a_1 = 0.004; % 4mm^2
a_2 = 0.004; % 4mm^2
R = 0.05;   % 5cm 
A = pi*R^2;
g = 9.98;
K_m = 0.0007; 

%Global Parameters
global theta_1 
global theta_2
global b
global h_max
theta_1 = a_1/A*sqrt(2*g);
theta_2 = a_2/A*sqrt(2*g);
b = K_m/A;
h_max = 0.1273; %10^(-3)/A


%Extra Parameters
global h_1e
global h_2e
global u_e
global K_x
global K_r
h_1e = 0.01;
h_2e = (theta_2/theta_1)^2*h_1e;
u_e = (theta_1/b)*sqrt(h_1e);
%% Simulation

% Gain matrix Calculation
K_x(1) = 100;
K_x(2) = 100;
% K_r = 255;
K_r = u_e/h_2e;
%Non Linear Plant State feedback Control
tspan = [0 10];
h0 = [0,0];
opt = odeset('Events',@overflow,'RelTol',1e-11,'AbsTol',1e-9);

[t,y] = ode23(@odefun3,tspan,h0,opt);

%% Plot

figure
clf
subplot(2,1,1)
plot(t,y(:,1))
hold on
plot(t,h_1e.*ones(size(t)),'k--')
grid on
subplot(2,1,2)
plot(t,y(:,2))
hold on
plot(t,h_2e.*ones(size(t)),'k--')
grid on
figure
clf
u = -K_x(1)*(y(:,1)-h_1e)-K_x(2)*(y(:,2)-h_2e)+K_r*h_2e;
plot(t,u)