% Final_project main script
clc;
clear all;

ur5 = ur5_interface();
ur5.init_gripper();

ur5.move_joints(ur5.home,5);
pause(5);


% Collecting input for grab positions
w = waitforbuttonpress;
gst_1 = ur5.get_current_transformation('base_link','tool0');
offset = [0,0,0,0;0,0,0,0;0,0,0,0.15;0,0,0,0];
%gst_1 = [0,-1,0,0.3;-1,0,0,-0.4;0,0,-1,0.22;0,0,0,1];

%gst_1 = [0,-1,0,-0.4;-1,0,0,0.6;0,0,-1,0.22;0,0,0,1];


% Testing
%ur5.move_joints(grab_joints_1,5);
%pause(5);

% Collecting input for placement position
w = waitforbuttonpress;
gst_2 = ur5.get_current_transformation('base_link','tool0');
%gst_2 =[0,-1,0,0.3;-1,0,0,0.39;0,0,-1,0.22;0,0,0,1];

%gst_2 =[0,-1,0,0.4;-1,0,0,0.6;0,0,-1,0.22;0,0,0,1];
 
% Testing where it is
%ur5.move_joints(place_joints_1,5);
%pause(5);

prev_config = ur5.home;
% Buidling path based off of inputs
gst_struct{1} = [gst_1 + offset];
gst_struct{2} = gst_1;
gst_struct{3} = [gst_1 + offset];
gst_struct{4} = [gst_2 + offset];
gst_struct{5} = gst_2;
gst_struct{6} = [gst_2 + offset];

for j = 1:size(gst_struct,2)
    % Computing inverse kinematics
    output = ur5InvKin(gst_struct{j});

    sorted_2 = zeros(6,4);
    count = 1;
    % Sorting based on the shoulder angle (joint 2)
    for i = 1:size(output,2)
        if output(2,i) < 0
            sorted_2(:,count) = output(:,i);
            count = count + 1;
        end
    end
    
    % Sorting based on base angle (joint 1)
    diff = zeros(1,size(sorted_2,2));
    for i = 1:size(sorted_2,2)
        diff(i) = abs(sorted_2(1,i) - prev_config(1));
    end
    
    % Finding index of minimum first joint
    [~,ind] = min(diff);
    vals = sorted_2(1,ind);
    
    % Taking values where this occurs in invKin output
    found = find(sorted_2(1,:)==vals);
    sorted_3 = zeros(6,length(found));
    for i = 1:length(found)
        sorted_3(:,i) = sorted_2(:,found(i));
    end

    % Finding sum of differences in joints 3,4,5,6
    diff_home = zeros(1,size(sorted_3,2));
    for i = 1:size(sorted_3,2)
        diff_home(i) = sum(abs(sorted_3(:,i) - prev_config));
    end
    [~,best_ind] = min(diff_home);

    best_config = sorted_3(:,best_ind);

    % Moving slower for father movement
    if j == 4
        ur5.move_joints(best_config,10);
        pause(10);
        w = waitforbuttonpress;
    else
        ur5.move_joints(best_config,5);
        pause(5);
        w = waitforbuttonpress;
    end
    % Close gripper here
    if j == 2
        ur5.close_gripper();
        pause(2);
    end
    % Let go of object here
    if j == 5
        ur5.open_gripper();
        pause(2);
    end
    % Save previous configuration
    prev_config = best_config;
end       

% return to home configuration
ur5.move_joints(ur5.home,5);
pause(5);

