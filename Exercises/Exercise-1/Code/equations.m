function dx = equations(t,x)
%% State spece form of the nonlinear model
    global fa
    global fe
    global k
    dx = [x(2); x(1)*x(4)^2 - k/x(1)^2 + fa; x(4);
        -(2*x(4)*x(2))/x(1) + (1/x(1))*fe];
end

    