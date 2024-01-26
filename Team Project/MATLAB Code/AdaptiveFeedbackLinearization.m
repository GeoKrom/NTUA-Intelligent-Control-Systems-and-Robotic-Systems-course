function dstate = AdaptiveFeedbackLinearization(t, state)
    %ADAPTIVEFEEDBACKLINEARIZITION Summary of this function goes here
    %   Detailed explanation goes here
    
    global P
    global b
    global a1m
    global a2m

    K1 = a2m;
    K2 = a1m;
    h1 = state(1);
    h2 = state(2);
    zm = state(3:4);
    theta1_hat = state(5);
    theta2_hat = state(6);

    
    % Input State Linearization
    z1 = h2;
    z2 = theta1_hat*sqrt(h1) - theta2_hat*sqrt(h2);
   
    

    dzm = [0 1; -a2m -a1m]*zm + [0; a2m]*ref(t);

    [h2_d, h2_d_dot, h2_d_ddot] = ref(t);
    
    e = [z1 - h2_d; z2 - h2_d_dot];

    u0 = h2_d_ddot - K1*e(1) - K2*e(2);
    H1 = [sqrt(h1); -0.5*theta1_hat-0.5*theta2_hat*sqrt(h1/h2)];
    H2 = [-sqrt(h2); 0.5*theta2_hat];
    % Adaptive Control Laws
    gamma1 = 0.8;
    gamma2 = 0.9;
    dtheta1 = gamma1*transpose(H1)*P*e;
    dtheta2 = gamma2*transpose(H2)*P*e;
    
    alpha = -dtheta1*sqrt(h1) + dtheta2*sqrt(h2) + 0.5*theta2_hat*(theta1_hat*sqrt(h1/h2) - theta2_hat);
    
    beta = theta1_hat*sqrt(h1)/b;

    u = (alpha + u0)*2*sqrt(h1)/(b*theta1_hat) + beta;

    dplant = plantDE(t, [h1;h2], u);
    dstate = [dplant; dzm; dtheta1; dtheta2];
end

