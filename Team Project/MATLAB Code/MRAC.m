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
    global h_max

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
    Gamma_x = 90000*eye(2);
    gamma_r = 50000;
    gamma_theta = 0;
    dKx = -Gamma_x*(x*e'*P*[b;0] + sigma*Kx_est);
    dKr = -gamma_r*(r*e'*P*[b;0] + sigma*Kr_est);
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

    u_c = Kx_est'*x + Kr_est*r;
    u = u_c + u_e;
    % h_safe = 0.18;
    % if h1 > h_safe
    %     u = u - ((h1 - h_safe)/(h_max - h_safe))*u;
    % end

    plant = plantDE(t, [h1;h2], u);

    
    dstate = [plant; dxm; dKx; dKr; dtheta];
end

