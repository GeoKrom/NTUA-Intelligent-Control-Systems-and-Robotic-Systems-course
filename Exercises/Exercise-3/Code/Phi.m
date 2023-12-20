function out = Phi(x,n_col)
    % Neural Network RBF Summary of this function goes here
    %   Detailed explanation goes here
    global K

    w = zeros(K,1);
    lr = 0.001;
    T_L = 1e-3.*cos(x).*sin(x);
    X_train = x;
    Y_train = T_L;
    epoch = 500;
    [nrow, ncol] = size(X_train);
    [idx, mi] = kmeans(X_train,K);
    out_rbf = zeros(size(X_train,1), K);
    
    max_dist_mi = [];
    for i = 1:1:K
        if i >= 1 && i < K
            for l = i+1:1:K
                max_dist_mi = [max_dist_mi; norm((mi(i,:) - mi(l,:)),2)];
            end
        end
    end
    disp("Computed maximum distances.....");
    disp(" ")
    
    dmax = max(max_dist_mi);
    sigma = dmax/sqrt(2*K);
    phi = (1/lr)*eye(K);
    
    
    
    for i=1:1:n_col
        xk = X_train(i,:);
        phin = zeros(K,1);
        for j = 1:K
            disp("Computing RBF Kernels....");
            disp(" ");
            phin(j) = exp(-(1/(2*(sigma^2)))*(norm((xk - mi(i,:)),2)^2));
            out_rbf(i,1) = phin(j);
        end
        disp("RBF Computed......");
        disp(" ");
        alphan = Y_train(i) - w'*phin;
        Pnew = phi - (phi*(phin*phin')*phi')/(1+phin'*phi*phin);
        wnew = w + Pnew.*phin.*alphan;
        w = wnew;
        phi = Pnew;
        T_L = wnew'*phin;  
        disp("......");
        disp(" ");
    end
   out = out_rbf;
end