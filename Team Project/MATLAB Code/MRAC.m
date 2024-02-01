function dstate = MRAC(t, state)
%MRAC Summary of this function goes here
%   Detailed explanation goes here
    global Am
    global Bm
    global a1m
    global a2m
    global P
    global h_2e
    global theta_1
    global theta_2
    global u_e
    global b


    h1 = state(1);
    h2 = state(2);
    xm = state(3:4);
    Kx_est = state(5:6);
    Kr_est = state(7);
    theta_est = state(8);

    r = ref(t) - h_2e;
    dot_h2 = theta_1*sqrt(h1) - theta_2*sqrt(h2);
    dxm = Am*xm + Bm*r;
    x = [h2 - h_2e; dot_h2];
    e = x - xm;
   
    % Adaptation Laws
    sigma = 0;
    Gamma_x = 500*eye(2);
    gamma_r = 100;
    gamma_theta = 0;
    dKx = -Gamma_x*(x*e'*P*[0;1] + sigma*Kx_est);
    dKr = -gamma_r*(r*e'*P*[0;1] + sigma*Kr_est);
    dtheta = -gamma_theta*e'*P*[0;1];
    
    %Projection Operator
    if Kr_est <= 0 && dKr < 0
        dKr = 0;
    end
    if Kr_est >= 1000 && dKr > 0
        dKr = 0;
    end
    
    if Kx_est(1) >= 0 && dKx(1) > 0
        dKx(1) = 0;
    end
    if Kx_est(1) <= -1000 && dKx(1) < 0
        dKx(1) = 0;
    end
    
    if Kx_est(2) >= 0 && dKx(2) > 0
        dKx(2) = 0;
    end
    if Kx_est(2) <= -1000 && dKx(2) < 0
        dKx(2) = 0;
    end

    u = Kx_est'*x + Kr_est*r;

    plant = plantDE(t, [h1;h2], u + u_e);

    
    dstate = [plant; dxm; dKx; dKr; dtheta];
end

