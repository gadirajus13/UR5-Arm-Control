function [perr, Rerr]=ur5TransJac_control(gst_struct,ur5)
    q_initial = [0 -pi/2 pi/2 -pi/2 -pi/2 pi/2]';    % give starting position
    ur5.move_joints(q_initial, 5);                         % move to that start position
    pause(5);
    K = 1;
    for i = 1:6
        [perr, Rerr] = ur5TransJac(gst_struct{i}, K, ur5); % call the UR55 control script
        display("Translational Error: "+perr);
        display("Rotational Error: "+Rerr);
    end

end