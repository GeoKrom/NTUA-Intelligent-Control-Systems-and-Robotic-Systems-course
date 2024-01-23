function dstate = linearplantDE1(t,state,u)
%Function to simulate the plant
global theta_1 
global theta_2
global b
global h_1e
global h_2e
global u_e
h_1 = state(1);
h_2 = state(2);
z_1 = h_1 - h_1e;
z_2 = h_2 - h_2e;

omega_1 = theta_1*(0.5/sqrt(h_1e));
omega_2 = theta_2*(0.5/sqrt(h_2e));



dz_1 = -omega_1*z_1+b*(u-u_e);
dz_2 =omega_1*z_1-omega_2*z_2;


dstate = [dz_1;dz_2];
end

