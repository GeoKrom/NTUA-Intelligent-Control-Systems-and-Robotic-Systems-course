function dstate = plantDE(t,state,u)
%Function to simulate the plant
global theta_1 
global theta_2
global b
global h_max
h_1 = state(1);
h_2 = state(2);
dh_1 = -theta_1*sqrt(h_1)+b*u;
dh_2 = theta_1*sqrt(h_1)-theta_2*sqrt(h_2);

% Projection of the Natural system
if h_1==h_max && dh_1>0
    dh_1=0;
elseif h_1==0 && dh_1<0
    dh_1=0;
end
if h_2==h_max && dh_2>0
    dh_2=0;
elseif h_2==0 && dh_2<0
    dh_2=0;
end

dstate = [dh_1;dh_2];
end

