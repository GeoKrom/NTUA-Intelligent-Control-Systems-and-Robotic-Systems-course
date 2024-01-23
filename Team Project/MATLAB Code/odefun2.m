function dstate = odefun2(t,state)
%Function To Simulate the Open Loop System
global u_e
if t==0
    u = 0;
else
    u = u_e;
end
plantdstate = linearplantDE1(t,state,u);
dstate = plantdstate;
end

