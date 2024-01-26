function projectionOperator = Proj(K, y, k_max, e_tol_bound)
%   Projection Summary of this function goes here
%   Detailed explanation goes here
    if(f(K, k_max, e_tol_bound) > 0 && y'*df(K, k_max, e_tol_bound) > 0)
        projectionOperator = (y - ((df(K, k_max, e_tol_bound)*(df(K, k_max, e_tol_bound))')/(norm(df(K, k_max, e_tol_bound))^2))...
                            *y*f(K, k_max, e_tol_bound));
    else
        projectionOperator = y;
    end
end

