function dstate = FeedbackLinearization(t, state)
%FEEDBACKLINEARIZITION Summary of this function goes here
%   Detailed explanation goes here

    global theta_1
    global theta_2
    global b
    global a2m
    
    h1 = state(1);
    h2 = state(2);
    
    % Input State Linearization
    K1 = 1;
    K2 = 10;
    z1 = h2;
    z2 = theta_1*sqrt(h1) - theta_2*sqrt(h2);
    [h2_d, h2_d_dot, h2_d_ddot] = ref(t);

    u0 = h2_d_ddot - K1*(z1 - h2_d) - K2*(z2 - h2_d_dot);

    alpha = -theta_1*theta_2*sqrt(h1/h2)/2 - theta_1^2/2 + theta_2^2/2;
    beta = b*theta_1/(2*sqrt(h1));

    u = (-alpha + u0)/beta;

    dplant = plantDE(t, [h1;h2], u);
    
    dstate = dplant;
end

