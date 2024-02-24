function dstate = Backstepping(t, state)
% BACKSTEPPING Summary of this function goes here
%   Detailed explanation goes here
    
    global theta_1
    global theta_2
    global b
    global a2m

    h1 = state(1);
    h2 = state(2);
    K1 = a2m;
    K2 = 2*sqrt(a2m);
    [h2_d, h2_d_dot, h2_d_ddot] = ref(t);

    ksi1 = h2 - h2_d;
    ksi2 = K1*ksi1 + theta_1*sqrt(h1) - theta_2*sqrt(h2) - h2_d_dot;
    
    alpha = K1^2*ksi1 - K2*ksi2 + 0.5*(theta_1^2 - theta_2^2) + 0.5*theta_1*theta_2*sqrt(h1/h2) - theta_2*h2_d_dot/(2*sqrt(h2)) + h2_d_ddot;
    
    beta = 2*sqrt(h1)/(b*theta_1);
    u = alpha*beta; 
    %u = u_c + u_e;
    % h_safe = 0.18;
    % if h1 >= h_safe
    %     u = u - ((h1 - h_safe)/(h_max - h_safe))*u;
    % end
    plant = plantDE(t, [h1;h2], u);
    
    dstate = plant;
end

