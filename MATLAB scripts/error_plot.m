% This script calculates the mean error between the estimate velocity and
% the velocity obtained from vicon's position measures.

begin_date = cust_time(1);
end_date = cust_time(end);
new_time = [];
new_pos_x = [];
new_pos_y = [];
new_pos_z = [];
new_vel_x = [];
new_vel_y = [];
new_vel_z = [];

for i=1:length(int_time)
    if(isbetween(int_time(i), begin_date, end_date))
        % adapted internal position
        new_time = [new_time; int_time(i)];
        new_pos_x = [new_pos_x; int_px(i)];
        new_pos_y = [new_pos_y; int_py(i)];
        new_pos_z = [new_pos_z; int_pz(i)];
        % adapted internal velocities
        new_vel_x = [new_vel_x; int_vx(i)];
        new_vel_y = [new_vel_y; int_vy(i)];
        new_vel_z = [new_vel_z; int_vz(i)];
    end
end

%% Position error
% interpolated vicon position
int_drone_posx = interp1(cust_time, drone_posx, new_time);
int_drone_posy = interp1(cust_time, drone_posy, new_time);
int_drone_posz = interp1(cust_time, drone_posz, new_time);

% x-position
pos_x = interp1(new_time, new_pos_x, new_time);
error_px = abs(int_drone_posx - pos_x);
mean_error_px = mean(error_px);

% y-velocity
pos_y = interp1(new_time, new_pos_y, new_time);
error_py = abs(int_drone_posy - pos_y);
mean_error_py = mean(error_py);

% z-velocity
pos_z = interp1(new_time, new_pos_z, new_time);
error_pz = abs(int_drone_posz - pos_z);
mean_error_pz = mean(error_pz);

%% Velocity error
%interpolated vicon velocities
int_drone_velx = interp1(cust_time, drone_vel_x, new_time);
int_drone_vely = interp1(cust_time, drone_vel_y, new_time);
int_drone_velz = interp1(cust_time, drone_vel_z, new_time);

% x-velocity
vx = interp1(new_time, new_vel_x, new_time);
error_vx = abs(int_drone_velx - vx);
mean_error_vx = mean(error_vx);

% y-velocity
vy = interp1(new_time, new_vel_y, new_time);
error_vy = abs(int_drone_vely - vy);
mean_error_vy = mean(error_vy);

% z-velocity
vz = interp1(new_time, new_vel_z, new_time);
error_vz = abs(int_drone_velz - vz);
mean_error_vz = mean(error_vz);


