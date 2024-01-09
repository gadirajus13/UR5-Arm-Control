% Loading in image
clc;
clear all;
filename = "Blue_jay.jpg";

original_image = imread(filename);

gr = rgb2gray(original_image);

% center = size(gr,2)/2;
% diff = size(gr,2) - size(gr,1);
% cut_image = gr(:,(diff/2)+1:size(gr,2)-(diff/2));

resize_image = imresize(gr,0.5);

% Adjust threshold for different images
outline = edge(resize_image,"canny",0.3);
imagesc(outline)

[rows,cols] = find(outline == 1);

% Offset to move off the page a centimeter
offset = [0,0,0,0;0,0,0,0;0,0,0,0.01;0,0,0,0];
% Sorting into lists where all parts don't need to be lifted
lines_struct = {};
lines_struct{1} = [rows(1),cols(1)];
struct_size = [1];

pick_up = [];
add = 0;
prev_coords = [rows(1),cols(1)];
sorted = [rows(1),cols(1)];
rows(1) = 1000;
cols(1) = 1000;
for j = 1:length(rows)
    
    dists = zeros(1,length(rows));
    for i = 1:length(rows)
        if i == j
            dists(i) = 1000;
        else
            dist = norm(prev_coords-[rows(i),cols(i)]);
            dists(i) = dist;
        end
    end
    [apart,ind] = min(dists);
    if apart > 3 
        pick_up = [pick_up,j];
    end
    sorted = [sorted;rows(ind),cols(ind)];
    prev_coords = [rows(ind),cols(ind)];
    rows(ind) = 1000;
    cols(ind) = 1000;
end

% Create gst_theta frames for each part of the line that is not on same row or column
gst_struct = {};
new_pick_up = [];
count = 1;
prev_sorted = zeros(1,2);
for i = 1:size(sorted,1)-2
    for j = 1:length(pick_up)
        % Creating frames in path so robot will pick up between different lines
        if i == (pick_up(j)+1) || i == 1 && j == 1
            gst_prev = createFrame(prev_sorted);
            gst_up = gst_prev + offset;
            gst_struct{count} = gst_up;
            new_pick_up = [new_pick_up,count,count+1,count+2];
            count = count + 1;
            gst_now = createFrame(sorted(i,:));
            gst_above = gst_now + offset;
            gst_struct{count} = gst_above;
            count = count + 1;        
        end
    end
    % Creating frames at each pixel for the robot to follow
    gst_next = createFrame(sorted(i,:));
    gst_struct{count} = gst_next;
    count = count + 1;
    prev_sorted = sorted(i,:);
end

ur5 = ur5_interface();

% Move robot to corner of image, then press button to draw
w = waitforbuttonpress;
gst_init = ur5.get_current_transformation('base_link','tool0');
prev_config = ur5.get_current_joints();

% For testing in simulation, uncomment this
%gst_init = [0,-1,0,0.3;-1,0,0,0.3;0,0,-1,0.22;0,0,0,1];

for i = 1:length(gst_struct)
    gst_struct{i}(1:3,1:3) = gst_struct{i}(1:3,1:3) * gst_init(1:3,1:3);
    gst_struct{i}(1:3,4) = gst_struct{i}(1:3,4) + gst_init(1:3,4);
end

cnt = 1;
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
    if (j == new_pick_up(cnt))
        ur5.move_joints(best_config,5);
        pause(5);
        cnt = cnt + 1;
     else
        ur5.move_joints(best_config,0.2);
        pause(0.2);
    end
    %w = waitforbuttonpress;
    % Save previous configuration
    prev_config = best_config;
end       

% return to home configuration
ur5.move_joints(ur5.home,10);
pause(10);

