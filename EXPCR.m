function [exp_x] = EXPCR(vec)
% Exponentiates the input vectors skew matrix, to create a rotation matrix
% that represents a rotation ||x|| about the axis n = x/||x||
% w*theta = (x/||x||)*(||x||) = x
x = SKEW3(vec);
mag_x = norm(vec);
exp_x = eye(3) + (sin(mag_x)/mag_x) * x + ((1-cos(mag_x))/(mag_x^2)) * x^2;

end