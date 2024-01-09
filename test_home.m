% Testing home frame

ur5 = ur5_interface();

ur5.move_joints(ur5.home,5);
pause(5);

gst_0 = ur5.get_current_transformation('base_link','ee_link');
pause(1);
disp("gst_0")
disp(gst_0)

joint_vec = [0; -pi/2; pi/2; 0; 0; pi/2]; % joint variables
[gst_theta_calc] = ur5FwdKin(joint_vec); % Forward kinemtics function
disp("Forward Kinematic Test 3a:")
disp("gst(theta)")
disp(gst_theta_calc) % Display for output

% Displaying and moving frame in ROS
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4)); % Creating testing frame
pause(1);
fwdKinToolFrame.move_frame('base_link',gst_theta_calc);% Moving testing frame
pause(1);
% Moving robot to final configuration
ur5.move_joints(joint_vec,5); % Moving ur5 to match
pause(5);
% Seeing what ROS says for gst_theta
gst_theta_ur5 = ur5.get_current_transformation('base_link','ee_link');
pause(1);
disp("Calculated by UR5")
disp(gst_theta_ur5)