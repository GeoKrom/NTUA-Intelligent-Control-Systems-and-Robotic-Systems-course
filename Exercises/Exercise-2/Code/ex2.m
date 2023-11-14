%%  MIT Rule Simulation
%   Assignment 2
%   Exercise 2
%       Name:   George Krommydas
%       A.M.:   02121208

clear;
clc;

% Parameter setting

global am
global bm
global b
global r
global gamma

am = 5;
bm = 12;           
b = 3;
tspan = [0 40];
n = 6;
x0 = zeros(n,1);
gamma = 0.5;

%% Simulation

[t, x] = ode45(@MITClosedLoop, tspan, x0);


%% Plots
theta1_desired = (bm/b).*ones(size(t)); 
theta2_desired = (am/b).*ones(size(t));
figure(1);
clf;
subplot(2,1,1);
plot(t,x(:,1),'b--');
hold on
plot(t, x(:,2), 'r-');
ylabel("Amplitude","Interpreter","latex", "FontSize", 12);
xlabel("$time [sec]$","Interpreter","latex", "FontSize", 12);
grid minor;
legend("$y_m$","$y_p$","Location","northeast","Interpreter","latex");

subplot(2,1,2);
plot(t, x(:,2) - x(:,1),"r-");
hold on
plot(t, zeros(size(t)), "b--");
xlabel("$time [sec]$","Interpreter","latex", "FontSize", 12);
grid minor;
ylabel("$e_0$","Interpreter","latex", "FontSize", 12);



figure(2);
clf;
plot(t,x(:,3),"r-");
hold on
plot(t, x(:,4), "b-");
plot(t, theta1_desired, "g--");
plot(t, theta2_desired, "k--");
xlabel("$time [sec]$", "Interpreter","latex", "FontSize", 12);
grid minor;
ylabel("Controller Gains","Interpreter","latex", "FontSize", 12);
legend("$\vartheta_1$","$\vartheta_2$","$\vartheta_1^*$","$\vartheta_2^*$","Location","southeast","Interpreter","latex");

%% Plant and Model Dynamics

function dx = MITClosedLoop(t, x)
    % Plant Dynamics
    
    global am
    global bm
    global b
    global r
    global gamma
    
    M = 4;
    r = M*sin(t)+M*sin(4*t)+M*sin(2*t);
    % r = 1;
    % r = sin(t);
    dx = [-am*x(1) + bm*r;
          -b*x(4)*x(2) + b*x(3)*r;
          -gamma*x(5)*(x(2) - x(1));
           gamma*x(6)*(x(2) - x(1));
           -am*x(5) + am*r;
           -am*x(6) + am*x(2)];   
end