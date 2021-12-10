clear; close all; clc
ifcommand= 1;
%% File loading
name = "crazyfun__20211210_165011.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);
vicon = fullfile(path, '..\vicon_data\', name); 
if(ifcommand)
command = fullfile(path, '..\command_data\', name); 
end
delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
raw_vicon_data = importdata(vicon,delimiterIn,headerlinesIn);
if(ifcommand)
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
if(ifcommand)
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
if(ifcommand)
command_vel_x = command_data(2:end,1);
command_vel_y = command_data(2:end,2);
command_vel_z = command_data(2:end,3);
command_vel_x_0 = command_data(1,1);
command_vel_y_0 = command_data(1,2);
command_vel_z_0 = command_data(1,3);

command_time = datetime(command_data(2:end,end), 'ConvertFrom', 'datenum');
end

int_int_px = interp1(int_time, int_px, int_time);
%int_drone_posx = interp1(cust_time, drone_posx, new_time);

clear vicon_data internal_data

%% Comparison between Euler angles
% This section analyzes the drone's internal attitude estimation with the
% Vicon-captured angles

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Comparison between Euler angles")
else
    figure2('name', "Comparison between Euler angles")
end

drone_vel_x = compute_vicon_vel(drone_posx);
drone_vel_y = compute_vicon_vel(drone_posy);
drone_vel_z = compute_vicon_vel(drone_posz);

subplot(3,1,1)
hold on
grid on
plot(cust_time, drone_vel_x,'r')
plot(int_time, int_vx,'b')
if(ifcommand)
plot(command_time, command_vel_x, 'g --')
end
ylabel("m/s")
legend('Vicon', 'Estimate')
title("Vx")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_vel_y,'r')
plot(int_time, int_vy,'b')
if(ifcommand)
plot(command_time, command_vel_y, 'g --')
end
ylabel("m/s")
legend('Vicon', 'Estimate')
title("Vy")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_vel_z,'r')
plot(int_time, int_vz,'b')
if(ifcommand)
plot(command_time, command_vel_z, 'g --')
end
ylabel("m/s")
legend('Vicon', 'Estimate')
title("Vz")

%% Integration of Velocity Command 
if(ifcommand)
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
if(ifcommand)
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
if(ifcommand)
plot(command_time,pos_comm_from_vel_y,'c')
end
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_posz,'r ')
plot(int_time, int_pz,'b')
if(ifcommand)
plot(command_time,pos_comm_from_vel_z,'c')
end
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Z coordinates")

%% Error between internal and vicon data
% This section analyzes the estimate difference between internal data (from
% EKF) and vicon data 
% 
% if exist('figure2') == 0  %#ok<*EXIST>
%     figure('name', "Error between positions")
% else
%     figure2('name', "Error between positions")
% end
% 
% subplot(3,1,1)
% hold on
% grid on
% plot(cust_time, error_x,'r')
% %plot(int_time, int_px,'b')
% ylabel("meter [m]")
% title("Error in X coordinates")
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(cust_time, error_y,'r')
% %plot(int_time, int_py,'b')
% ylabel("meter [m]")
% title("Error in Y coordinates")
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(cust_time, error_z,'r')
% %plot(int_time, int_pz,'b')
% ylabel("meter [m]")
% title("Error in Z coordinates")

