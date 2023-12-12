function dstate = MRACStepperNeuralNetwork(t,state)
    
    global B
    global J
    global Km
    global r
    global count
    global P
    global Am
    global Q
    global Bm



    period = t(:,1);
    disp("Time Loop t = " + period + "sec");
    r0 = 0.087266;
    
    % Parameter Uncertainties
    per = 0.0;
    B_un = (B - B*per) + ((B + B*per) - (B - B*per))*rand(1);
    J_un = (J - J*per) + ((J + J*per) - (J - J*per))*rand(1);
    Km_un = (Km - Km*per) + ((Km + Km*per) - (Km - Km*per))*rand(1);
    
    % Plant State Space Matrices
    A = [0 1; 
         0 -B_un/J_un];
    b = [0; 
         Km_un/J_un];
    % Adaptation Gains
    g3 = 2;
    Gx = [5 0;
          0 1];
    Gtheta = [1 0 0;
              0 2.5 0;
              0 0 3];
    % Increment input signal
    if (count == floor(period/3))
        r = r + r0;
        count = count + 1;
        disp("Input has increase by 5 deg!");
    end
    
    % States
    state = state(:);

    x = state(1:2);
    x_m = state(3:4);
    Kx_est = state(5:6);
    Kr_est = state(7);
    Ktheta_est = state(8:K);
    T_L = 1e-3*((cos(2*x(1))).^2).*sin(3*x(1));     % Nm
    % Estimated Error
    e = x - x_m;
    % disp("Calcuted Error");
   
    % Simulation of the system
    % Control Signal
    % u = Kx_est'*x + Kr_est*r;
    u = Kx_est'*x + Kr_est*r + Ktheta_est'*NN(x);
    % dx = A*x + b*u;
    dx = A*x + b*u - T_L;
    dxm = Am*x_m + Bm*r;

    % Adaptive laws
    % dKx = -Gx*x*e'*P*b;
    % dKr = -g3*r*e'*P*b; 
    dKx = Gx*Proj(Kx_est, -x*e'*P*b, 1, 1e-5);
    dKr = g3*Proj(Kr_est, -r*e'*P*b, 5, 1e-4);
    dKtheta = Gtheta*Proj(Ktheta_est, -NN(x)*e'*P*B, 10, 1e-4);
    dstate = [dx; dxm; dKx; dKr; dKtheta];
end

