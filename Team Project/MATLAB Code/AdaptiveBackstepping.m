function dstate = AdaptiveBackstepping(t, state)
    %ADAPTIVEBACKSTEPPING Summary of this function goes here
    %   Detailed explanation goes here
    
    global b
    
    h1 = state(1);
    h2 = state(2);
    theta1_hat = state(3);
    theta2_hat = state(4);
    
    K1 = 10;
    K2 = 5;
    [h2_d, h2_d_dot, h2_d_ddot] = ref(t);
    
    % Diffeomorphism Transformation
    ksi1 = h2 - h2_d;
    ksi2 = K1*ksi1 + theta1_hat*sqrt(h1) - theta2_hat*sqrt(h2) - h2_d_dot;

    alpha2 = K1*(-K1*ksi1 + ksi2) - h2_d_ddot + 0.5*(theta2_hat^2 - theta1_hat^2) - 0.5*theta1_hat*theta2_hat*sqrt(h1/h2);
    beta1 = K1*sqrt(h1) - 0.5*theta1_hat - 0.5*theta2_hat*sqrt(h1/h2);
    beta2 = -K1*sqrt(h2) + 0.5*theta2_hat;

    % Adaptation Laws
    gamma1 = 0.5;
    gamma2 = 0.6;

    dtheta1 = gamma1*(ksi1*sqrt(h1) + ksi2*beta1);
    dtheta2 = gamma2*(-ksi1*sqrt(h2) + ksi2*beta2);
    
    % Control Input 
    alpha = -ksi1 - alpha2 - dtheta1*sqrt(h1) + dtheta2*sqrt(h2) - K2*ksi2;
    beta = 2*sqrt(h1)/(b*theta1_hat);
    
    u = alpha/beta;
    plant = plantDE(t, [h1;h2], u);
    
    dstate = [plant; dtheta1; dtheta2];

end

