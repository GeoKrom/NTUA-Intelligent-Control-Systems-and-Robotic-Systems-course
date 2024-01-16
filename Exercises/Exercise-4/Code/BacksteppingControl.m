function dstate = BacksteppingControl(t, state)
%  BACKSTEPPING CONTROL Summary of this function goes here
%   Detailed explanation goes here
    global m1
    global l1
    global Iz1
    global k1
    global m2
    global l2
    global Iz2
    global k2
    global g
    global parvar

    % Desired trajectories
    q1_d = 90*pi/180 + (30*pi/180)*cos(t);
    q1_dot_d = -(30*pi/180)*sin(t);
    q1_ddot_d = -(30*pi/180)*cos(t);
    q2_d = 90*pi/180 - (30*pi/180)*sin(t);
    q2_dot_d = -(30*pi/180)*cos(t);
    q2_ddot_d = (30*pi/180)*sin(t);
    
    % Gain matrices
    K1 = [3.5 0;
          0 .8];
    K2 = [15 0;
          0 15];

    q_d = [q1_d;
           q2_d];
    q_dot_d = [q1_dot_d;
               q2_dot_d];
    q_ddot_d = [q1_ddot_d;
                q2_ddot_d];
    
    % States
    state = state(:);
    q1 = state(1);
    q2 = state(2);
    q1_dot = state(3);
    q2_dot = state(4);
    
    q = [q1;
         q2];

    q_dot = [q1_dot;
             q2_dot];
    
    c1 = cos(q1);
    c2 = cos(q2);
    c12 = cos(q1 + q2);
    s2 = sin(q2);
    
    q_dot_r = q_dot_d + K1*(q - q_d);
    q_ddot_r = q_ddot_d + K1*(q_dot - q_dot_d);
    
    % Mass Matrix
    M11 = Iz1 + Iz2 + m1*(l2^2/4) + m2*(l1^2 +l2^2/4 + l1*l2*c2);
    M12 = Iz2 + m2*(l2^2/4 + l1*l2*c2);
    M22 = Iz2 + m2*l2^2/4;
    M = [M11 M12;
         M12 M22];

    % Coriolis Matrix
    c = 0.5*m2*l1*l2*s2;
    C11 = -c*q2_dot + k1;
    C12 = -c*(q1_dot + q2_dot);
    C21 = c*q1_dot;
    C22 = k2;
    C = [C11 C12;
         C21 C22];

    % Gravity vector
    G1 = 0.5*m1*g*l1*c1 + m2*g*(l1*c1 + 0.4*l2*c12);
    G2 = 0.5*m2*g*l2*c12;
    G_0 = [G1;
           G2];
    
    Y = [q_ddot_r(1) q_ddot_r(1)+q_ddot_r(2) c2*(2*q_ddot_r(1)+q_ddot_r(2))-s2*(q_dot(2)*q_dot_r(1)+(q_dot(1)+q_dot(2)*q_dot_r(2))) q_dot_r(1) 0 c1 c12;
        0 q_ddot_r(1)+q_ddot_r(1) c2*q_ddot_r(1)+s2*q_dot(1)*q_dot_r(1) 0 q_dot_r(2) 0 c12];

    % Control law
    z1 = q - q_d;
    e2 = q_dot - q_dot_d;
    z2 = e2 + K1*z1;
    tau = Y*parvar - z1 - K2*z2;
    q_ddot = inv(M)*(tau - C*q - G_0);

    dstate = [q_dot; q_ddot];
end

