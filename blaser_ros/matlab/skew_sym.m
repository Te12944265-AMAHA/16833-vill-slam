function [X] = skew_sym(x)
%SKEW_SYM Summary of this function goes here
%   Detailed explanation goes here
X = [    0, -x(3),  x(2);
      x(3),     0, -x(1);
     -x(2),  x(1),    0];
end

