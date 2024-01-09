function Jb = ur5BodyJacobian(q) 
% Computes the Body Jacobian of UR5 robot
% INPUTS
% q: Takes input of a vector of joint variables in radians
% OUTPUTS
% A 6x6 Body Jacobian of the UR5 robot

% -------------------------------------------------------------------------
% ----- Defining all internal parameters (twists, link lenghts, gst0) -----
% -------------------------------------------------------------------------

% Define Link Lengths of UR5 Robot (units - m) 
% dimensions based on provided figure from RDKDC HW6
L0 = 0.0892; L1 = 0.425; L2 = 0.3923; 
L3 = 0.1093; L4 = 0.09475; L5 = 0.0825;

% Define all the points on the twist axes 
q0 = [0; 0; L0]; q1 = [0;0;L0]; q2 = [L1; 0; L0]; 
q3 = [L1+L2; 0; L0]; q4 = [L1+L2;L3;L0]; q5 = [L1+L2;L3;L0-L4];

% Define all the rotation vectors 
w1 = [0;0;1]; w2 = [0;1;0]; w3 = [0;1;0]; 
w4 = [0;1;0]; w5 = [0;0;-1]; w6 = [0;1;0];

% Calculate all the twists
Xi1 = [-cross(w1, q0); w1];
Xi2 = [-cross(w2, q1); w2];
Xi3 = [-cross(w3, q2); w3];
Xi4 = [-cross(w4, q3); w4];
Xi5 = [-cross(w5, q4); w5];
Xi6 = [-cross(w6, q5); w6];
Xi = [Xi1 Xi2 Xi3 Xi4 Xi5 Xi6];

% Define the home configuration gst(0)
gst0 = [[0,1,0;1,0,0;0,0,-1],[L1+L2; L3 + L5; L0-L4]; 0 0 0 1];

% -------------------------------------------------------------------------
% -------------------- Computing the Body Jacobian ------------------------
% -------------------------------------------------------------------------

PoE = gst0;         % define Product of Exponentials "dummy" variable that will update in for loop
Jb = ones([6, 6]);  % Define Body Jacobian 6x6 matrix that is filled with ones

for i = 6:-1:1      % iterating from n = 6 down to i (inverse for loop)
    % Find the  
    XiHat = [SKEW3(Xi(4:6,i)) [Xi(1, i); Xi(2, i); Xi(3, i)]
            0 0 0 0];
    % find the matrix exponential of the skew-symmetric form of twist
    % g = exp^(Xihat*theta) formula is used 
    gexp = expm(XiHat.*q(i)); 
    PoE = gexp*PoE;         % This PoE variable gets updated as we from twist 6 down to twist 1
    
    % setup the Inverse Adjoint Matrix 
    Rot = PoE(1:3,1:3);     % Extract the Rotational Component of PoE output
    p = PoE(1:3, 4);        % Extract the Translational Component of PoE output
    phat = SKEW3(p);        % Convert the "p" translation into skew-symmetric matrix
    
    % Compute the Inverse Adjoint 6x6 Matrix 
    Adj_inv = [Rot' -Rot'*phat;
          zeros([3, 3]) Rot']; 

    % Multiply the Inverse Adjoint Matrix with the initial twist
    % Append the twist to the body Jacobian matrix
    Jb(:,i) = Adj_inv*Xi(:,i); 
end 

end 