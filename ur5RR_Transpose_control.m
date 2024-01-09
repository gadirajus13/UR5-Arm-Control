function [finalerr] = ur5TransJac(gdesired, K, ur5)
%offset = ur5.get_current_transformation('ee_link', 'gripper_pick');
%pause(1);
%offsetR = offset(1:3,1:3);
%gdesired(1:3,4) = gdesired(1:3,4) - offset(1:3,4);
%gdesired(1:3,1:3) = gdesired(1:3,1:3)*offsetR;

% define continuation thresholds
Rtarget = gdesired(1:3,1:3);
vthresh = .03; %.04 thresh works with inv(Jb), .3 worked updated
wthresh = 1*(pi/180);
tstep = [0.7:-0.03:0.01];

% find initial error and v, w
greal = ur5.get_current_transformation('base_link','ee_link');
pause(1);
% xdesired = getXi(greal);
% wdesired = xdesired(4:6);

error = FINV(gdesired)*greal;
Xi = getXi(error);
v = Xi(1:3);
w_diff = 10;
count = 1;
close = 1;
_control
% repeat until v and w are below thresholds
while ((norm(v) >= vthresh) || (w_diff >= wthresh)) && count <= length(tstep)
        
    q = ur5.get_current_joints(); %get current joint angles
    pause(1);

    Jb = ur5BodyJacobian(q); %find jacobian of current joint angles
    q2 = q - K*tstep(count)*transpose(Jb)*Xi; %use given formula from class to find the 
    % next joint angles to get to gdesired
    
    %if we are close to a singularity, return -1 and break
    if abs(det(Jb)) < 0.001
        finalerr = -1;
        display(finalerr);
        pause(1);
        ur5.move_joints([0; -pi/2; pi/2; 0; -pi/5; pi/2], 20); %arbitrary safe location
        pause(20);
        return;
    end
    
    %deal with hitting table case
    %g2 = ur5FwdKin(q2);
    if greal(3,4)<.0001 %maybe this needs to be changed?
        finalerr = -2;
        disp("HIT TABLE");
        return;
    end
    
    % Dealing with second joint being positive
    if q2(2) > 0
        finalerr = -2;
        disp("HIT TABLE")
        return;
    end
    
    %move the robot to the next calculated location
    ur5.move_joints(q2, 5);
    pause(5);
    
    %update error
    greal = ur5.get_current_transformation('base_link','ee_link');
    pause(1);
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
    disp(norm(v))
    disp(norm(w_diff))
    count = count + 1;
end

%calculate and return final error
greal = ur5.get_current_transformation('base_link','ee_link');
pause(1);
finalerr = norm(greal(1:3, 4) - gdesired(1:3, 4))*100;
end