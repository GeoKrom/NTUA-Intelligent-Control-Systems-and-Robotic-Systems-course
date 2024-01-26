function out = df(theta, theta_max, e_theta_tol_bound)
%DF Summary of this function goes here
%   Detailed explanation goes here
    out = (2*theta)/(e_theta_tol_bound*theta_max^2);
end

