function [xi] = getXi(g)
R = g(1:3,1:3); %get rotation matrix from input transformation

%calculate w and theta
theta = acos((sum(diag(R)) - 1)/2);
if theta < .03
    w = [0;0;0];
    p = g(1:3,4);
    v= p;%/norm(p);
    xi = [v;w];
else
    w = (1/(2*sin(theta)))*[R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    w_hat = SKEW3(w); %use SKEW3 function to find the skew of the w we found
    A = (eye(3) - R)*w_hat + w*w'*theta; %formula from class, Av = p
    b = g(1:3, 4); %get b as p from the original transformation
    v = A\b; %mldivide(A, b); %find v as Av = b

    xi = [v; w]; %return the screw coordinates
end
end