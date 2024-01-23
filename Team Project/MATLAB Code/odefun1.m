function dstate = odefun1(t,state)
%Function To Simulate the Open Loop System
global u_e
if t==0
    u = 0;
else
    u = u_e;
end
plantdstate = plantDE(t,state,u);
dstate = plantdstate;
end

