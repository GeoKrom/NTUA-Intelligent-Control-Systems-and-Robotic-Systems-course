%% Stepper Motor Adaptive Control
%   Assignment 3
%       Name: George Krommydas
%       A.M.: 02121208

clear;
clc;

global B
global J
global Km
global r
global count
global P
global Am
global Q
global Bm

%% Parameter setting
B = 8*1e-4;                 % Nm sec/rad
J = 4.5*1e-5;               % kgr m^2
Km = 0.19;                  % N m/A
N = 50;
Am = [0 1; 
     -24 -10];
Bm = [0;
      24];
Q = eye(2);
P = lyap(Am',Q);

tstart = 0;
tend = 50;
tspan = [tstart, tend];
n = 7;
x0 = zeros(n,1);
opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-6);
count = 1;
K = 5;
r = 0.087266;


%% Neural Network Training






%% Ode simulation

[t,x] = ode45(@MRACStepper, tspan, x0, opt);

% Ideal Gains
K_theta_star = -24*(J/Km).*ones(size(t));
K_omega_star = (10 - B/J)*(J/Km).*ones(size(t));
K_theta_c_star = -K_theta_star;

% Plots
figure(1);
clf;

subplot(2,1,1);
plot(t, x(:,1), 'r-');
hold on;
plot(t, x(:,3), 'b-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\vartheta [rad]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\vartheta$', '$\vartheta_r$','Interpreter','latex',Location='southeast');

subplot(2,1,2);
plot(t, x(:,2),'r-');
hold on;
plot(t, x(:,4), 'b-');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$\omega [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;
legend('$\omega$', '$\omega_r$','Interpreter','latex',Location='northeast');


figure(2);
clf;

subplot(2,1,1);
plot(t, x(:,1)-x(:,3), 'r-');
hold on;
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{\vartheta} [rad]$','Interpreter','latex','FontSize',14);
grid on;
subplot(2,1,2);
plot(t, x(:,2)-x(:,4), 'r-');
hold on;
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$e_{\omega} [rad/sec]$','Interpreter','latex','FontSize',14);
grid on;

figure(3);
clf;

subplot(3,1,1);
plot(t, x(:,5), 'r-');
hold on;
plot(t, K_theta_star,'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$K_{\vartheta}$','Interpreter','latex','FontSize',14);
grid on;
legend('$\hat{K}_{\vartheta}$','$K_{\vartheta}^*$','Interpreter','latex',Location='northeast');

subplot(3,1,2);
plot(t, x(:,6), 'r-');
hold on;
plot(t, K_omega_star,'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$K_{\omega}$','Interpreter','latex','FontSize',14);
grid on;
legend('$\hat{K}_{\omega}$','$K_{\omega}^*$','Interpreter','latex',Location='northeast');

subplot(3,1,3);
plot(t, x(:,7), 'r-');
hold on;
plot(t, K_theta_c_star,'b--');
xlabel('$time [sec]$','Interpreter','latex','FontSize',14);
ylabel('$K_{\vartheta_{c}}$','Interpreter','latex','FontSize',14);
grid on;
legend('$\hat{K}_{\vartheta_c}$','$K_{\vartheta_c}^*$','Interpreter','latex',Location='northeast');