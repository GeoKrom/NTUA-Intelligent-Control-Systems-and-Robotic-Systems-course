function [value,isterminal,direction]=overflow(t,x)
global h_max
value(1) = x(1)-h_max;
value(2) = x(2)-h_max;
isterminal(1) = 1;
isterminal(2) = 1;
direction(1) = 0;
direction(2) = 0;
end