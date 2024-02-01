function dstate = AdaptiveFeedbackLinearization(t, state)
    %ADAPTIVEFEEDBACKLINEARIZITION Summary of this function goes here
    %   Detailed explanation goes here
    
    global P
    global b
    global a1m
    global a2m

    K1 = 1;
    K2 = 10;
    h1 = state(1);
    h2 = state(2);
    zm = state(3:4);
    theta1_hat = state(5);
    theta2_hat = state(6);
    b_est = state(7);
    
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
    gamma1 = 0.00035;
    gamma2 = 0.00025;
    dtheta1 = gamma1*transpose(H1)*P*e;
    dtheta2 = gamma2*transpose(H2)*P*e;
    
    %Projection
    if theta1_hat <= 0.00001 && dtheta1 < 0
     dtheta1 = 0;
    end
    
    if theta2_hat <= 0.00001 && dtheta2 < 0
        dtheta2 = 0;
    end

    alpha = -dtheta1*sqrt(h1) + dtheta2*sqrt(h2) + 0.5*theta2_hat*(theta1_hat*sqrt(h1/h2) - theta2_hat);
    
    beta = theta1_hat*sqrt(h1)/b_est;

    u = (alpha + u0)*2*sqrt(h1)/(b_est*theta1_hat) + beta;
    
    gamma3 = 0.0055;
    H3 = [0; theta1_hat*(1/(2*sqrt(h1)))*u];
    dbeta = gamma3*H3'*P*e;
    
    if b_est <= 0.00001 && dbeta < 0
        dbeta = 0;
    end
    
    dplant = plantDE(t, [h1;h2], u);
    dstate = [dplant; dzm; dtheta1; dtheta2; dbeta];
end

