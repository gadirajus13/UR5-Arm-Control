% % RDKDC Resolve Rate Control

% Collecting input for placement position
w = waitforbuttonpress;
gdesired = ur5.get_current_transformation('base_link','ee_link');

% q_initial = [pi/2; -pi/4; pi/4; -pi/4; -pi/2; pi/2];    % give starting position
% %q_initial = [pi/2; pi/2; pi/3; pi/2; pi/2; pi/8];      %THIS IS THE QINIT TO HIT THE TABLE
% ur5.move_joints(q_initial, 5);                         % move to that start position
% pause(5);
% 
% %q_target = [pi/2 + pi/2; -pi/4; pi/4; -pi/4; -pi/2; pi/2];  % give target position
% %q_target = [0; -pi/2; pi/2; 0; -pi/2; pi/2];
% q_target = [0; -pi/3; pi/2; 0; 0; pi/4];
% gst = ur5FwdKin(q_target);                                  % compute the forward kinematics
% gdesired = gst; 
pause(2);
desiredFrame = tf_frame('base_link', 'desiredFrame', gdesired); % label target pos frame 
pause(2); 

K = 1;
w = waitforbuttonpress;
finalerr = ur5RRcontrol(gdesired, K, ur5);              % call the UR55 control script
display(finalerr)

%%
%singularity
% ur5.move_joints(ur5.home, 50);
% pause(50);
% 
% gdesired = [eye(3) [-.15; -.16; -.55]; 0 0 0 1];
% desiredFrame = tf_frame('base_link', 'desiredFrame', gdesired);
% pause(5);
% 
% K = 1;
% finalerr = ur5RRcontrol(gdesired, K, ur5);
% -------------------------------------------------------------------------
% -------------------------- -----------------------------------------------