function [gst_theta] = ur5FwdKin(q)
% Computes the forward kinematics of UR5 robot
% INPUTS
% q: Takes input of a vector of joint variables in radians
% OUTPUTS
% gst: a 4 x 4 matrix that is the end effector pose

% -------------------------------------------------------------------------
% ----- Defining all internal parameters (twists, link lenghts, gst0) -----
% -------------------------------------------------------------------------

%initial positions - dimensions based on provided figure from RDKDC HW6 and
%RVIZ
joint_vec = q;      % re-define joint_vec to remove confusion with "q" axes vectors 
L0 = 0.0892; L1 = 0.425; L2 = 0.3923; L3 = 0.1093; L4 = 0.09475; L5 = 0.0825;

% Home Configuration 
gst_0 = [[0,1,0;1,0,0;0,0,-1],[L1+L2; L3 + L5; L0-L4]; 0 0 0 1]; %[0,1,0;1,0,0;0,0,-1]

% Define Axes of Rotation
w1 = [0;0;1]; w2 = [0;1;0]; w3 = [0;1;0]; 
w4 = [0;1;0]; w5 = [0;0;-1]; w6 = [0;1;0];

% Define twist axis points "q"
q0 = [0; 0; L0]; q1 = [0;0;L0]; q2 = [L1; 0; L0]; q3 = [L1+L2; 0; L0]; q4 = [L1+L2;L3;L0]; 
q5 = [L1+L2;L3;L0-L4];

% Finding v 
v1 = cross(-w1,q0);
v2 = cross(-w2,q1);
v3 = cross(-w3,q2);
v4 = cross(-w4,q3);
v5 = cross(-w5,q4);
v6 = cross(-w6,q5);

% -------------------------------------------------------------------------
% -------------------- Computing the Forward Kinematics -------------------
% -------------------------------------------------------------------------

% Finding the twist exponential, calling twist_exp_real function
etwist1 = twist_exp_real(w1, v1, joint_vec(1));
etwist2 = twist_exp_real(w2, v2, joint_vec(2));
etwist3 = twist_exp_real(w3, v3, joint_vec(3));
etwist4 = twist_exp_real(w4, v4, joint_vec(4));
etwist5 = twist_exp_real(w5, v5, joint_vec(5));
etwist6 = twist_exp_real(w6, v6, joint_vec(6));

% Product of Exponential Formula to compute the gst (end-effector pose) 
gst_theta = etwist1 * etwist2 * etwist3 * etwist4 * etwist5 * etwist6 * gst_0;

end