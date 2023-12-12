function out = f(theta, theta_max, e_theta_tol_bound)
%F Summary of this function goes here
%   Detailed explanation goes here
    out = (norm(theta)^2 - theta_max^2)/(e_theta_tol_bound*theta_max^2);
end

