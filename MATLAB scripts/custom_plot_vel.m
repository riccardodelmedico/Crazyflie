clear; close all; clc
command_flag = 1;

%% File loading
name = "crazyfun__20211210_165011.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);
vicon = fullfile(path, '..\vicon_data\', name); 
delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
raw_vicon_data = importdata(vicon,delimiterIn,headerlinesIn);
if(command_flag)
    command = fullfile(path, '..\command_data\', name); 
    raw_command_data = importdata(command,delimiterIn,headerlinesIn);
end

if isstruct(raw_internal_data)
    internal_data = raw_internal_data.data;
else
    internal_data = raw_internal_data;
end

if isstruct(raw_vicon_data)
    vicon_data = raw_vicon_data.data;
else
    vicon_data = raw_vicon_data;
end
if(command_flag)
    if isstruct(raw_command_data)
        command_data = raw_command_data.data;
    else
        command_data = raw_command_data;
    end
end

clear vicon internal command
clear raw_vicon_data raw_internal_data raw_command_data
clear current_file delimiterIn headerlinesIn name path

%% Data extraction
% Extracted data                        Variables meaning 
drone_posx = vicon_data(:,1);           % \
drone_posy = vicon_data(:,2);           %  |-> drone position from Vicon, in Vicon frame [m]
drone_posz = vicon_data(:,3);           % /
% drone_vel_x = vicon_data(:,4);          %  \
% drone_vel_y = vicon_data(:,5);          %   |-> drone orientation through quaternions from Vicon 
% drone_vel_z = vicon_data(:,6);          %  /          % /
cust_time = datetime(vicon_data(:,end), 'ConvertFrom', 'datenum');


int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_pz = internal_data(:,3);            % /
int_vx = internal_data(:,4);          % \
int_vy = internal_data(:,5);         % |-> internal estimate of drone attitude
int_vz = internal_data(:,6);           % /
int_time = datetime(internal_data(:,end), 'ConvertFrom', 'datenum');

if(command_flag)
    command_vel_x = command_data(2:end,1);
    command_vel_y = command_data(2:end,2);
    command_vel_z = command_data(2:end,3);
    command_vel_x_0 = command_data(1,1);
    command_vel_y_0 = command_data(1,2);
    command_vel_z_0 = command_data(1,3);

    command_time = datetime(command_data(2:end,end), 'ConvertFrom', 'datenum');
end

%int_int_px = interp1(int_time, int_px, int_time);
%int_drone_posx = interp1(cust_time, drone_posx, new_time);

clear vicon_data internal_data

%% Comparison between velocities
if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Comparison between Velocities")
else
    figure2('name', "Comparison between Velocities")
end

drone_vel_x = compute_vicon_vel(drone_posx);
drone_vel_y = compute_vicon_vel(drone_posy);
drone_vel_z = compute_vicon_vel(drone_posz);

subplot(3,1,1)
hold on
grid on
plot(cust_time, drone_vel_x,'r')
plot(int_time, int_vx,'b')
if(command_flag)
    plot(command_time, command_vel_x, 'g --')
    legend('Vicon', 'Estimate', 'Velocity Command')
end
ylabel("m/s")
title("Vx")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_vel_y,'r')
plot(int_time, int_vy,'b')
if(command_flag)
    plot(command_time, command_vel_y, 'g --')
    legend('Vicon', 'Estimate', 'Velocity Command')
end
ylabel("m/s")
title("Vy")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_vel_z,'r')
plot(int_time, int_vz,'b')
if(command_flag)
    plot(command_time, command_vel_z, 'g --')
    legend('Vicon', 'Estimate', 'Velocity Command')
end
ylabel("m/s")
title("Vz")

%% Integration of Velocity Command 
if(command_flag)
    pos_comm_from_vel_x = integrated_command_velocity(command_vel_x,command_vel_x_0);
    pos_comm_from_vel_y = integrated_command_velocity(command_vel_y,command_vel_y_0);
    pos_comm_from_vel_z = integrated_command_velocity(command_vel_z,command_vel_z_0);
end

%% Comparison between positions
% This section analyzes the drone's internal position estimation with the
% Vicon-captured position

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Comparison between positions")
else
    figure2('name', "Comparison between positions")
end

subplot(3,1,1)
hold on
grid on
plot(cust_time, drone_posx,'r ')
plot(int_time, int_px,'b')
if(command_flag)
plot(command_time,pos_comm_from_vel_x,'c')
end
%plot(cust_time, interp_posx, 'g')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_posy,'r')
plot(int_time, int_py,'b ')
if(command_flag)
    plot(command_time, pos_comm_from_vel_y, 'c')
end
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_posz,'r ')
plot(int_time, int_pz,'b')
if(command_flag)
    plot(command_time, pos_comm_from_vel_z, 'c')
end
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Z coordinates")

%% Error between internal and vicon data
% This section ...
error_plot;

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Error between velocities")
else
    figure2('name', "Error between velocities")
end
 
subplot(2,1,1)
hold on
grid on
plot(new_time, error_vx,'r')
ylabel("m/s")
title("Error in Vx")

subplot(2,1,2)
hold on
grid on
plot(new_time, error_vy,'r')
ylabel("m/s")
title("Error in Vy")
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(cust_time, error_z,'r')
% %plot(int_time, int_pz,'b')
% ylabel("meter [m]")
% title("Error in Z coordinates")
