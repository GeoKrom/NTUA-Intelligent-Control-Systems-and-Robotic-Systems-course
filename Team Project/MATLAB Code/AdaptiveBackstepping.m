function dstate = AdaptiveBackstepping(t, state)
    % ADAPTIVEBACKSTEPPING Summary of this function goes here
    %   Detailed explanation goes here
    
    global a2m
    global b
    
    h1 = state(1);
    h2 = state(2);
    theta1_hat = state(3);
    theta2_hat = state(4);
    b_est = state(5);

    K1 = sqrt(a2m);
    K2 = 2*K1;
    [h2_d, h2_d_dot, h2_d_ddot] = ref(t);
    
    % Diffeomorphism Transformation
    ksi1 = h2 - h2_d;
    ksi2 = K1*ksi1 + theta1_hat*sqrt(h1) - theta2_hat*sqrt(h2) - h2_d_dot;

    alpha2 = K1*(-K1*ksi1 + ksi2) - h2_d_ddot + 0.5*(theta2_hat^2 - theta1_hat^2) - 0.5*theta1_hat*theta2_hat*sqrt(h1/h2);
    beta1 = K1*sqrt(h1) - 0.5*theta1_hat - 0.5*theta2_hat*sqrt(h1/h2);
    beta2 = -K1*sqrt(h2) + 0.5*theta2_hat;
   
    % Adaptation Laws
    gamma1 = 0.001;
    gamma2 = 0.001;
    gamma3 = 0.001;
    dtheta1 = gamma1*(ksi1*sqrt(h1) + ksi2*beta1);
    dtheta2 = gamma2*(-ksi1*sqrt(h2) + ksi2*beta2);
    
    
    % Projection
    if theta1_hat <= 0.00001 && dtheta1 < 0
        dtheta1 = 0;
    end
    if theta2_hat <= 0.00001 && dtheta2 < 0
        dtheta2 = 0;
    end

    % Control Input 
    alpha = -ksi1 - alpha2 - dtheta1*sqrt(h1) + dtheta2*sqrt(h2) - K2*ksi2;
    beta = 2*sqrt(h1)/(b_est*theta1_hat);
    u = alpha*beta;
    
    beta3 = theta1_hat*(1/(2*sqrt(h2)))*u;
    dbeta = gamma3*ksi2*beta3;
    
    if b_est <= 0.00001 && dbeta < 0
        dbeta = 0;
    end

    plant = plantDE(t, [h1;h2], u);
    
    dstate = [plant; dtheta1; dtheta2; dbeta];

end

