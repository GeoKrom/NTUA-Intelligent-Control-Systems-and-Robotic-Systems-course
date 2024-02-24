%% Control of a Multiple Tank System
%  Mesuarement Analysis
%  Adaptive Feedback Linearization Controller

clc;
clear;


T = readtable("output_linear.csv");
disp(T);

figure(1);
clf;

subplot(2,1,1);
plot(T.val,10^(-3)*T.h1,'r-');
xlabel("$time [sec]$","Interpreter","latex");
ylabel("$h_1 [m]$","Interpreter","latex");
grid on;

subplot(2,1,2);
plot(T.val,10^(-3)*T.h2,'r-');
xlabel("$time [sec]$","Interpreter","latex");
ylabel("$h_2 [m]$","Interpreter","latex");
grid on;

% figure(2);
% clf;
% 
% subplot(3,1,1);
% plot(T.val,T.theta1,'r-');
% xlabel("$time [sec]$","Interpreter","latex");
% ylabel("$\theta_1$","Interpreter","latex");
% grid on;
% 
% subplot(3,1,2);
% plot(T.val,T.theta2,'r-');
% xlabel("$time [sec]$","Interpreter","latex");
% ylabel("$\theta_2$","Interpreter","latex");
% grid on;
% 
% subplot(3,1,3);
% plot(T.val,T.theta3,'r-');
% xlabel("$time [sec]$","Interpreter","latex");
% ylabel("$b$","Interpreter","latex");
% grid on;