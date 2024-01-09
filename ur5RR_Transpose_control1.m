function [finalerr] = ur5RR_Transpose_control(gdesired, K, ur5)
%offset = ur5.get_current_transformation('ee_link', 'gripper_pick');
%pause(1);
%offsetR = offset(1:3,1:3);
%gdesired(1:3,4) = gdesired(1:3,4) - offset(1:3,4);
%gdesired(1:3,1:3) = gdesired(1:3,1:3)*offsetR;

% define continuation thresholds
Rtarget = gdesired(1:3,1:3);
vthresh = .1; %.04 thresh works with inv(Jb), .3 worked updated
wthresh = 1*(pi/180);

% find initial error and v, w
greal = ur5.get_current_transformation('base_link','ee_link');
pause(1);
% xdesired = getXi(greal);
% wdesired = xdesired(4:6);
dist_trav = norm(abs(gdesired(1:3,4)-greal(1:3,4)));
step_list = [0.14,0.13,0.12,0.11,0.1,0.09,0.08,0.07,0.06,0.05,0.04,0.03,0.02,0.01];
tstep = step_list * dist_trav;

error = FINV(gdesired)*greal;
Xi = getXi(error);
v = Xi(1:3);
w_diff = 10;
count = 1;
close = 1;

%(norm(v) >= vthresh)

% repeat until v and w are below thresholds
while (w_diff >= wthresh) && count <= 14 %% Needs to be initial tstep length
        
    q = ur5.get_current_joints(); %get current joint angles
    pause(0.3);

    Jb = ur5BodyJacobian(q); %find jacobian of current joint angles
    q2 = q - K*tstep(count)*transpose(Jb)*Xi; %use given formula from class to find the 
    % next joint angles to get to gdesired
    disp(q2)
    
    %if we are close to a singularity, return -1 and break
    if abs(det(Jb)) < 0.001
        finalerr = -1;
        display(finalerr);
        %pause(1);
        ur5.move_joints([0; -pi/2; pi/2; 0; -pi/5; pi/2], 20); %arbitrary safe location
        pause(20);
        return;
    end
    
    %deal with hitting table case
    g2 = ur5FwdKin(q2);
    gripperFrame = ur5.get_current_transformation('ee_link','gripper_pick');
    next_frame = g2*gripperFrame;
    if next_frame(3,4)<.005 %maybe this needs to be changed?
        finalerr = -2;
        disp("GOING TO HIT TABLE");
        return;
    end
    
    % Dealing with second joint being positive
    if q2(2) > 0
        finalerr = -2;
        disp("HIT TABLE")
        return;
    end
    
    %move the robot to the next calculated location
    ur5.move_joints(q2, 2);
    pause(2);
    
    %update error
    greal = ur5.get_current_transformation('base_link','ee_link');
    pause(0.3);
    error = FINV(gdesired)*greal;
    Xi = getXi(error);
    v = Xi(1:3);
    
%     if norm(v) < 0.4 && close == 1
%         tstep = [.12:-.01:.01];
%         count = 0;
%         close = 0;
%     end

    Rreal = greal(1:3,1:3);
    Rrealt = Rreal'*Rtarget;
    w_diff = acos((sum(diag(Rrealt)) - 1)/2);
    count = count + 1;
    finalerr = norm(greal(1:3, 4) - gdesired(1:3, 4))*100;
    if finalerr > 2 && count == 14
        dist_trav = norm(abs(gdesired-greal));
        step_list = [0,0,0,0,0,0,0,0,0,0.5,0.3,0.2,0.1,0.04];
        tstep = step_list * dist_trav;
        count = 9;
    end
    
    if (abs(q(1)-q2(1)) < 0.02)
        q2(1) = q2(1) - 1;
    end
    disp(norm(v))
    disp(norm(w_diff))
end
        
%calculate and return final error
greal = ur5.get_current_transformation('base_link','ee_link');
pause(1);
finalerr = norm(greal(1:3, 4) - gdesired(1:3, 4))*100;
end