function [value,isterminal,direction]=overflow(t,x)
    global h_max
    
    value(1) = x(1)-h_max;
    value(2) = x(2)-h_max;
    value(3) = x(1);
    value(4) = x(2);
    isterminal(1) = 1;
    isterminal(2) = 1;
    isterminal(3) = 1;
    isterminal(4) = 1;
    direction(1) = 0;
    direction(2) = 0;
    direction(3) = 0;
    direction(4) = 0;
end