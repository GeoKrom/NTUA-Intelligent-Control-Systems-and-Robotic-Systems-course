%%  MIT Rule Simulation
%   Assignment 2
%   Exercise 1
%       Name:   George Krommydas
%       A.M.:   02121208

clear;
clc;

% Parameter setting

global a
global w
global zeta
global r
global gamma

a = -5;
w = 10;            % rad/sec
zeta = 1;
tspan = [0 50];
n = 13;
x0 = zeros(n,1);
gamma = 1;

%% Simulation

[t, x] = ode45(@MITClosedLoop, tspan, x0);


%% Plots
theta1_desired = (w^2).*ones(size(t)); 
theta2_desired = (w^2).*ones(size(t));
theta3_desired = (2*zeta*w-a).*ones(size(t));
figure(1);
clf;
subplot(2,1,1);
plot(t,x(:,1),'b--');
hold on
plot(t, x(:,3), 'r-');
ylabel("Amplitude","Interpreter","latex", "FontSize", 12);
xlabel("$time [sec]$","Interpreter","latex", "FontSize", 12);
grid minor;
legend("$y_m$","$y_p$","Location","northeast","Interpreter","latex");

subplot(2,1,2);
plot(t, x(:,3) - x(:,1),"r-");
hold on
plot(t, zeros(size(t)), "b--");
xlabel("$time [sec]$","Interpreter","latex", "FontSize", 12);
grid minor;
ylabel("$e_0$","Interpreter","latex", "FontSize", 12);



figure(2);
clf;
plot(t,x(:,5),"r-");
hold on
plot(t, x(:,6), "b-");
plot(t, x(:,7), "g-");
plot(t, theta1_desired, "Color", [0.3010 0.7450 0.9330], "LineStyle","--");
plot(t, theta2_desired, "Color", [0 0.4470 0.7410], "LineStyle","--");
plot(t, theta3_desired, "Color", [0.6350 0.0780 0.1840], "LineStyle","--");
xlabel("$time [sec]$", "Interpreter","latex", "FontSize", 12);
grid minor;
ylabel("Controller Gains","Interpreter","latex", "FontSize", 12);
legend("$\vartheta_1$","$\vartheta_2$","$\vartheta_3$","$\vartheta_1^*$","$\vartheta_2^*$","$\vartheta_3^*$","Location","northwest","Interpreter","latex");

%% Plant and Model Dynamics

function dx = MITClosedLoop(t, x)
    % Plant Dynamics
    
    global a
    global w
    global zeta
    global r
    global gamma
    
    % M = 4;
    % r = M*sin(t)+M*sin(4*t)+M*sin(2*t);
    % r = 1;
    r = sin(t);
    dx = [x(2);
          -w.^2*x(1) - 2*zeta*w*x(2) + w.^2.*r; 
          x(4);
         -x(6)*x(3) - (a + x(7))*x(4) + x(5).*r;
          -gamma*x(8)*(x(3) - x(1));
          gamma*x(10)*(x(3) - x(1));
          gamma*x(12)*(x(3) - x(1));
          x(9);
          -w^2*x(8) - 2*zeta*w*x(9) + w.^2.*r;
          x(11);
          -w^2*x(10) - 2*zeta*w*x(11) + w.^2.*x(3);
          x(13);
          -w^2*x(12) - 2*zeta*w*x(13) + w.^2.*x(4)];   
end
