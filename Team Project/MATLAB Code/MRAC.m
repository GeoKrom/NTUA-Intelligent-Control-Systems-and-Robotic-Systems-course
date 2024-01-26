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
    global a1_tilda
    global a2_tilda
    global b_tilda

    h1 = state(1);
    h2 = state(2);
    xm = state(3:4);
    Kx_est = state(5:6);
    Kr_est = state(7);
    theta_est = state(8);
    [h2_d, ~, ~] = ref(t);
    r = ref(t) - h_2e;
    dot_h2 = theta_1*sqrt(h1) - theta_2*sqrt(h2);
    dxm = Am*xm + Bm*r;
    a1_tilda = -theta_2 - (b*theta_1^2*u_e)/(4*theta_2^2*h2_d*sqrt(h2_d));
    a2_tilda = -(b*theta_1^2*u_e)/(2*theta_2^3*h2_d);
    b_tilda = (b*theta_1^2)/(2*theta_2*sqrt(h2_d));
    x = [h2 - h_2e; dot_h2];
    e = x - xm;
    
    u = Kx_est'*x + Kr_est*r + theta_est + u_e;

    plant = plantDE(t, [h1;h2], u);
    % Adaptation Laws
    Gamma_x = 100*eye(2);
    gamma_r = 100;
    gamma_theta = 1;
    dKx = -Gamma_x*x*e'*P*[0;1];
    dKr = -gamma_r*r*e'*P*[0;1];
    dtheta = -gamma_theta*e'*P*[0;1];
    dstate = [plant; dxm; dKx; dKr; dtheta];
end

