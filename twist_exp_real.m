function [out] = twist_exp_real(w,v,theta)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
% MLS Equation 2.36 - pg. 47
x = w * theta;
p = (eye(3) - EXPCR(x))*cross(w,v) + w*w'*v*theta;
R = EXPCR(x);
if theta == 0
    out = [eye(3),theta*v;0 0 0 1];
else
    out = [R,p;0,0,0,1];
end
end