%% Control of a Multiple Tank System
%       Team Project for course Intelligent Control Systems and Robotics.
%       Team 1
%       Names - A.M.:
%       George Kassavetakis - 02121203
%       George Krommydas - 02121208
%       Lampis Papakostas - 02121211
%       Control Of multiple tank system using these algorithms:
%           1. MIR Rule.
%           2. Model Reference Adaptive Control.
%           3. Feedback Linearization Control.
%           4. Backstepping Control.
%           5. Adaptive Feedback Linearization Control.
%           6. Adaptive Backstepping Control.

clc;
clear;
tic;

%% Parameter Settings

% Constant Parameters

a_1 = 0.004; % 4mm^2
a_2 = 0.004; % 4mm^2
R = 0.05;   % 5cm 
A = pi*R^2;
g = 9.81;
K_m = 0.0007; 

% Global Parameters

global theta_1 
global theta_2
global b
global h_max
global a1m
global a2m
global P
global Am
global Bm
global a1_tilda
global a2_tilda
global b_tilda

theta_1 = a_1/A*sqrt(2*g);
theta_2 = a_2/A*sqrt(2*g);
b = K_m/A;
h_max = 0.240; %10^(-3)/A
method = input("Please select your desired method: ");
a1m = 25;
a2m = 10;

% Extra Parameters

global h_1e
global h_2e
global u_e
global ref_sig
h_1e = h_max/2 - 0.01;
h_2e = (theta_2/theta_1)^2*h_1e;
u_e = (theta_1/b)*sqrt(h_1e);
ref_sig = 2;
tspan = [0 120];
opt = odeset('Events',@overflow,'RelTol',1e-11,'AbsTol',1e-9);
Am = [0 1; -a2m -a1m];
Bm = [0; a2m];
Q = eye(2);
P = lyap(Am', Q);


%% Simulation

if method == 1
    % MIT Control
    n = 13;
    h0 = zeros(n,1);
    h0(3) = -h_1e;
    h0(4) = -h_2e;
    
    [t,y] = ode23(@MITRule, tspan, h0, opt);
    
    % Plots
    
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, y(:,3)+h_2e,'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$x_2$','$x_m$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    figure(2);
    clf;
    
    subplot(3,1,1);
    plot(t, y(:,5),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,2);
    plot(t,y(:,6),'r-');
    
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,3);
    plot(t,y(:,7),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_3$','Interpreter','latex');
    legend('$\hat{\theta}_3$','Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;
    
elseif method == 2
    
    % MRAC Control
    n = 8;
    h0 = zeros(n,1);
    h0(1) = h_1e;
    h0(2) = h_2e;
    
    [t,y] = ode23(@MRAC, tspan, h0, opt);
    
    Kr_star = (a2m/b_tilda).*ones(size(t,1));
    Kx_star1 = ((-a2m - a1_tilda)/b_tilda).*ones(size(t,1));
    Kx_star2 = ((-a1m - a2_tilda)/b_tilda).*ones(size(t,1));
    % Plots
    
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, y(:,3),'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$x_2$','$x_m$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    figure(2);
    clf;
    
    subplot(4,1,1);
    plot(t, y(:,5),'r-');
    hold on;
    plot(t, Kx_star1, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_{x_1}$','Interpreter','latex');
    legend('$\hat{K}_{x_1}$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(4,1,2);
    plot(t,y(:,6),'r-');
    hold on;
    plot(t, Kx_star2, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_{x_2}$','Interpreter','latex');
    legend('$\hat{K}_{x_2}$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(4,1,3);
    plot(t,y(:,7),'r-');
    hold on;
    plot(t, Kr_star, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_r$','Interpreter','latex');
    legend('$\hat{K}_r$','Location','northeast',"Interpreter","latex");
    grid on;

    subplot(4,1,4);
    plot(t,y(:,8),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta$','Interpreter','latex');
    legend('$\hat{\theta}$','Location','northeast',"Interpreter","latex");
    grid on;
    
    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 3
    
    % Feedback Linearization Control
    n = 2;
    h0 = zeros(n,1);
    h0(1) = h_1e - 0.05;
    h0(2) = h_2e - 0.05;

    [t,y] = ode23(@FeedbackLinearization, tspan, h0, opt);


    % Plots
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    grid on;

    figure(2);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;
    

elseif method == 4
    
    % Adaptive Feedback Linearization Control
    
    n = 6;
    h0 = zeros(n,1);
    h0(1) = h_1e - 0.05;
    h0(2) = h_2e - 0.05;
    h0(3) = 2.2;
    h0(4) = 2.2;
    h0(5) = h0(2);
    h0(6) = 0;
    [t,y] = ode23(@AdaptiveFeedbackLinearization, tspan, h0, opt);

    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t, y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t, y(:,2),'r-');
    hold on;
    plot(t, y(:,3), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    grid on;


    figure(2);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,5),'r-');
    hold on;
    plot(t, theta_1.*ones(size(t,1)), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$','$\theta_{1_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,6),'r-');
    hold on;
    plot(t, theta_2.*ones(size(t,1)), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','$\theta_{2_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 5
    
    % Backstepping Control
    n = 2;
    h0 = zeros(n,1);
    h0(1) = h_1e - 0.05;
    h0(2) = h_2e - 0.05;

    [t,y] = ode23(@FeedbackLinearization, tspan, h0, opt);


    % Plots
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    grid on;
    
    figure(2);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 6
    
    % Adaptive Backstepping Control
    n = 4;
    h0 = zeros(n,1);
    h0(1) = h_1e - 0.05;
    h0(2) = h_2e - 0.05;
    h0(3) = 2.2;
    h0(4) = 2.2;
    [t,y] = ode23(@AdaptiveBackstepping, tspan, h0, opt);

    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t, y(:,1),'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t, y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    grid on;


    figure(2);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,3),'r-');
    hold on;
    plot(t, theta_1.*ones(size(t,1)), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$','$\theta_{1_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,4),'r-');
    hold on;
    plot(t, theta_2.*ones(size(t,1)), 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','$\theta_{2_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

else
    
    disp(" ");
    disp("You did not choose a specific controller");
    disp(" ");
    disp("Usage: >>Control_Tank_System ");
    disp("Correct numbers for variable 'method' ");
    disp("method = " + " 1 " + " | " + " 2 " + " | " + " 3 " + " | " + " 4 " + " | " + " 5 " + " | " + " 6 ");
    disp(" ");
    disp("Please selected a controller which is shown below:");
    disp(" ");
    disp("    1. MIT Rule Controller.");
    disp("    2. Model Reference Adaptive Controller.");
    disp("    3. Feedback Linearization Controller.");
    disp("    4. Adaptive Feedback Linearization Controller.");
    disp("    5. Backstepping Controller.");
    disp("    6. Adaptive Backstepping Controller.");
    disp(" ");
end
