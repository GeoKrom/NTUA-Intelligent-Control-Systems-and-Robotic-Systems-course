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

a_1 = pi*(2.0e-03)^2;   % 4 mm^2
a_2 = pi*(2.0e-03)^2;   % 4 mm^2
R = 0.04;              % 3.5 cm 
A = pi*R^2;
g = 9.81;               % m/sec^2
%K_m = 0.0000667/5;      % m^3/V sec 
K_m = 240*10^(-3)/(5*3600); 

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
global theta1_star
global theta2_star
global theta3_star


theta_1 = a_1/A*sqrt(2*g);
theta_2 = a_2/A*sqrt(2*g);

b = K_m/A;
h_max = 10^(-3)/A;
method = input("Please select your desired method: ");
a2m = .01;
a1m = 2*sqrt(a2m);

% Extra Parameters

global h_1e
global h_2e
global u_e
global ref_sig

h_1e = h_max/2;
h_2e = (theta_2/theta_1)^2*h_1e;
u_e = (theta_1/b)*sqrt(h_1e);
omega_1 = (a_1/A)*(g/(sqrt(2*g*h_1e)));
omega_2 = (a_2/A)*(g/(sqrt(2*g*h_2e)));

ref_sig = 1;
minutes = 60;
tspan = [0 minutes*60];
% opt = odeset('Events',@overflow,'RelTol',1e-11,'AbsTol',1e-9);
opt = odeset('RelTol',1e-11,'AbsTol',1e-9);
Am = [0 1; -a2m -a1m];
Bm = [0; a2m];
Q = eye(2);
P = lyap(Am', Q);
b_real = b;

%% Simulation

if method == 1
    % MIT Control
    theta1_star = (a2m/(b*omega_1));
    theta2_star = ((a1m - (omega_1 + omega_2))/b);
    theta3_star = ((a2m - omega_1*omega_2 - b*theta2_star*omega_2)/(b*omega_1));
    
    n = 13;
    state0 = zeros(n,1);
    state0(1) = h_1e;
    state0(2) = h_2e;
    state0(5) = 200;
    state0(6) = 60;  
    state0(7) = 150;
    [t,y] = ode23(@MITRule, tspan, state0, opt);

    
    % Plots
    
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, y(:,3)+h_2e,'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$x_2$','$x_m$','Location','northeast',"Interpreter","latex");
    grid on;
    

    figure(2);
    clf;
    
    subplot(3,1,1);
    plot(t, y(:,5),'r-');
    hold on;
    xlim(tspan);
    % yline(theta1_star, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$', '$\theta_1^*$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,2);
    plot(t,y(:,6),'r-');
    hold on;
    xlim(tspan);
    % yline(theta2_star, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','$\theta_2^*$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,3);
    plot(t,y(:,7),'r-');
    hold on;
    xlim(tspan);
    % yline(theta3_star, 'b--');
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_3$','Interpreter','latex');
    legend('$\hat{\theta}_3$','$\theta_3^*$','Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
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
    
    Kr_star = (a2m/b);
    Kx_star1 = ((-a2m - (omega_1*omega_2))/b);
    Kx_star2 = ((-a1m - (omega_1+omega_2))/b);
    % Plots
    
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, y(:,3)+h_2e,'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$x_2$','$x_m$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    figure(2);
    clf;
    
    subplot(3,1,1);
    plot(t, y(:,5),'r-');
    hold on;
    % yline(Kx_star1, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_{x_1}$','Interpreter','latex');
    legend('$\hat{K}_{x_1}$','$K_{x_1}^*$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,2);
    plot(t,y(:,6),'r-');
    hold on;
    % yline(Kx_star2, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_{x_2}$','Interpreter','latex');
    legend('$\hat{K}_{x_2}$','$K_{x_2}^*$','Location','northeast',"Interpreter","latex");
    grid on;
    
    
    subplot(3,1,3);
    plot(t,y(:,7),'r-');
    hold on;
    % yline(Kr_star, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$K_r$','Interpreter','latex');
    legend('$\hat{K}_r$','$K_r^*$','Location','northeast',"Interpreter","latex");
    grid on;
    
    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 3
    
    % Feedback Linearization Control
    n = 2;
    h0 = zeros(n,1);
    h0(1) = h_1e;
    h0(2) = h_2e;

    [t,y] = ode23(@FeedbackLinearization, tspan, h0, opt);


    % Plots
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$h_2$', '$x_m$', 'Location', 'northeast', 'Interpreter', 'latex');
    grid on;

    figure(2);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;
    

elseif method == 4
    
    % Adaptive Feedback Linearization Control
    
    n = 7;
    h0 = zeros(n,1);
    h0(1) = h_1e;
    h0(2) = h_2e;
    h0(3) = h_2e;
    h0(4) = 0;
    h0(5) = 0.0105;
    h0(6) = 0.0105;
    h0(7) = 0.0002;
    [t,y] = ode23(@AdaptiveFeedbackLinearization, tspan, h0, opt);
    
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t, y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t, y(:,2),'r-');
    hold on;
    plot(t, y(:,3), 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$h_2$', '$x_m$', 'Location', 'northeast', 'Interpreter', 'latex');
    grid on;


    figure(2);
    clf;
    
    subplot(3,1,1);
    plot(t,y(:,5),'r-');
    hold on;
    yline(theta_1, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$','$\theta_{1_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;
    
    subplot(3,1,2);
    plot(t,y(:,6),'r-');
    hold on;
    yline(theta_2, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','$\theta_{2_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;

    subplot(3,1,3);
    plot(t,y(:,7),'r-');
    hold on;
    yline(b_real, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$b$','Interpreter','latex');
    legend('$\hat{b}$','$b_d$', 'Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 5
    
    % Backstepping Control
    n = 2;
    h0 = zeros(n,1);
    h0(1) = h_1e;
    h0(2) = h_2e;

    [t,y] = ode23(@Backstepping, tspan, h0, opt);


    % Plots
    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t,y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$h_2$', '$x_m$', 'Location', 'northeast', 'Interpreter', 'latex');
    grid on;
    
    figure(2);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

elseif method == 6
    
    % Adaptive Backstepping Control
    n = 5;
    h0 = zeros(n,1);
    h0(1) = h_1e;
    h0(2) = h_2e;
    h0(3) = 0.01;
    h0(4) = 0.01;
    h0(5) = 0.002;
    [t,y] = ode23(@AdaptiveBackstepping, tspan, h0, opt);

    figure(1);
    clf;
    
    subplot(2,1,1);
    plot(t, y(:,1),'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_1 [m]$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t, y(:,2),'r-');
    hold on;
    plot(t, ref(t), 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$h_2 [m]$','Interpreter','latex');
    legend('$h_2$', '$x_m$', 'Location', 'northeast', 'Interpreter', 'latex');
    grid on;


    figure(2);
    clf;
    
    subplot(3,1,1);
    plot(t,y(:,3),'r-');
    hold on;
    % yline(theta_1, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_1$','Interpreter','latex');
    legend('$\hat{\theta}_1$','$\theta_{1_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;
    
    subplot(3,1,2);
    plot(t,y(:,4),'r-');
    hold on;
    % yline(theta_2, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$\theta_2$','Interpreter','latex');
    legend('$\hat{\theta}_2$','$\theta_{2_d}$', 'Location','northeast',"Interpreter","latex");
    grid on;

    subplot(3,1,3);
    plot(t, y(:,5),'r-');
    hold on;
    % yline(b_real, 'b--');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$b$','Interpreter','latex');
    legend('$\hat{b}$','$b_d$', 'Location','northeast',"Interpreter","latex");
    grid on;

    figure(3);
    clf;
    plot(t, y(:,2) - ref(t), 'r-');
    xlim(tspan);
    xlabel('$time [sec]$','Interpreter','latex');
    ylabel('$e [m]$','Interpreter','latex');
    grid on;

else
    
    disp(" ");
    disp("You did not chose a specific controller");
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