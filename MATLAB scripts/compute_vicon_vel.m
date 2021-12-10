function [vicon_vel] = compute_vicon_vel(vicon_pos)
    dt = 0.1; %[s] => vicon pos at 10Hz
    vicon_vel = zeros(length(vicon_pos), 1);
    for i=1:length(vicon_pos) - 1
        vicon_vel(i+1) = (vicon_pos(i+1) - vicon_pos(i))/dt;
    end
end

