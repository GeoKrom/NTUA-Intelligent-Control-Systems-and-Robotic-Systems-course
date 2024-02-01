function dstate = MITRule(t, state)
%MITRULE Summary of this function goes here
%   Detailed explanation goes here
    
    global h_1e
    global h_2e
    global u_e
    global a1m
    global a2m
    global Am
    global theta1_star
    global theta2_star
    global theta3_star
    
    r = ref(t) - h_2e;
    %r = h_2e/2; 
    
    Am = [0 1; -a2m -a1m];
    
    h_1 = state(1);
    h_2 = state(2);
    x_m = state(3:4);
    theta1_est = state(5);
    theta2_est = state(6);
    theta3_est = state(7);
    filter1 = state(8:9);
    filter2 = state(10:11);
    filter3 = state(12:13);
    
    x1 = h_1 - h_1e;
    x2 = h_2 - h_2e;
    
    gamma_1 = 500;
    gamma_2 = 180;
    gamma_3 = 150;
    
    dxm = Am*x_m + [0; a2m]*r;          
    dfilter1 = Am*filter1 + [0;a2m]*r;  
    dfilter2 = Am*filter2 + [0;a2m]*x1; 
    dfilter3 = Am*filter3 + [0;a2m]*x2;
    
    u = theta1_est*r - theta2_est*x1 - theta3_est*x2;
    % u = theta1_star*r - theta2_star*x1 - theta3_star*x2;
    plantdstate = plantDE(t, [h_1;h_2], u + u_e);
    
    
    e = x2 - x_m(1);
    theta_1_dot = - gamma_1*e*filter1(1);
    theta_2_dot = gamma_2*e*filter2(1);
    theta_3_dot = gamma_3*e*filter3(1);
    
    dstate = [plantdstate; dxm; theta_1_dot; theta_2_dot; theta_3_dot; dfilter1; dfilter2; dfilter3];

end

