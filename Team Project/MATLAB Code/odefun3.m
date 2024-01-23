function dstate = odefun3(t,state)
%Function To Simulate the Linear Feedback Closed Loop
global h_1e
global h_2e
global K_x
global K_r
h_1 = state(1);
h_2 = state(2);
r = h_2e;
u = -K_x(1)*(h_1-h_1e)-K_x(2)*(h_2-h_2e)+K_r*r;
plantdstate = plantDE(t,state,u);
dstate = plantdstate;
end

