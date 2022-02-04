clear; close all; clc
command_flag = 1;

%% File loading
name = "crazyfun__20220131_140348.txt";
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
int_vx = internal_data(:,4);            % \
int_vy = internal_data(:,5);            % |-> internal estimate of drone attitude
int_vz = internal_data(:,6);            % /
int_time = datetime(internal_data(:,end), 'ConvertFrom', 'datenum');

if(command_flag)
    ref_vel_x = command_data(2:end,1);
    ref_vel_y = command_data(2:end,2);
    ref_vel_z = command_data(2:end,3);
    ref_vel_x_0 = command_data(1,1);
    ref_vel_y_0 = command_data(1,2);
    ref_vel_z_0 = command_data(1,3);
    comm_vel_x = command_data(2:end,4);
    comm_vel_y = command_data(2:end,5);
    command_time = datetime(command_data(2:end,end), 'ConvertFrom', 'datenum');
end

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
    plot(command_time, ref_vel_x, 'g --')
    plot(command_time, comm_vel_x, 'm --')
    legend('Vicon', 'Estimate', 'Velocity Reference','Velocity Command')
end
ylabel("m/s")
title("Vx")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_vel_y,'r')
plot(int_time, int_vy,'b')
if(command_flag)
    plot(command_time, ref_vel_y, 'g --')
    plot(command_time, comm_vel_y, 'm --')
    legend('Vicon', 'Estimate', 'Velocity Reference','Velocity Command')
end
ylabel("m/s")
title("Vy")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_vel_z,'r')
plot(int_time, int_vz,'b')
if(command_flag)
    plot(command_time, ref_vel_z, 'g --')
    legend('Vicon', 'Estimate', 'Velocity Command')
end
ylabel("m/s")
title("Vz")

%% Integration of Velocity Command 

if(command_flag)
    pos_comm_from_vel_x = integrated_command_velocity(ref_vel_x,ref_vel_x_0);
    pos_comm_from_vel_y = integrated_command_velocity(ref_vel_y,ref_vel_y_0);
    pos_comm_from_vel_z = integrated_command_velocity(ref_vel_z,ref_vel_z_0);
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
plot(command_time, pos_comm_from_vel_x,'c')
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

%% Error between internal and vicon position
% This section analyzes the estimate error between internal estimate
% position and vicon position 
%position_error;

% Output of get_time will also be used for Velocity Error
[new_time, first_index, size] = get_time(cust_time, int_time);

% Compute the 'new' position used for error computation
% (nota: questo passo è necessario per avere un calcolo coerente
% dell'errore, dal momento che i dati interni del drone e i dati del vicon
% sono ottenuti per tempi diversi e a frequenza diversa
new_pos_x = int_px(first_index:first_index+size-1);
new_pos_y = int_py(first_index:first_index+size-1);
new_pos_z = int_pz(first_index:first_index+size-1);
new_pos = [new_pos_x, new_pos_y, new_pos_z];
drone_pos = [drone_posx, drone_posy, drone_posz];

[pos_error, mean_pos_error] = get_error(cust_time, new_time, ...
                                         drone_pos, new_pos);

fprintf("Mean Position Error:\n x: %f, y: %f, z: %f\n", ...
        mean_pos_error(1), mean_pos_error(2), mean_pos_error(3));
if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Error between position")
else
    figure2('name', "Error between position")
end

subplot(3,1,1)
hold on
grid on
plot(new_time, pos_error(:,1),'r')
ylabel("[m]")
title("Error in x")

subplot(3,1,2)
hold on
grid on
plot(new_time, pos_error(:,2),'r')
ylabel("[m]")
title("Error in y")

subplot(3,1,3)
hold on
grid on
plot(new_time, pos_error(:,3),'r')
ylabel("[m]")
title("Error in z")

%% Error between internal and vicon velocity
% This section analyzes the estimate error between internal estimate
% velocity and vicon velocity, obtained simply by taking time derivative of
% its position

% Compute the 'new' velocity used for error computation
new_vel_x = int_vx(first_index:first_index+size-1);
new_vel_y = int_vy(first_index:first_index+size-1);
new_vel_z = int_vz(first_index:first_index+size-1);
new_vel = [new_vel_x, new_vel_y, new_vel_z];
drone_vel = [drone_vel_x, drone_vel_y, drone_vel_z];

[vel_error, mean_vel_error] = get_error(cust_time, new_time, ...
                                         drone_vel, new_vel);

fprintf("Mean Velocity Error:\n x: %f, y: %f, z: %f\n", ...
        mean_vel_error(1), mean_vel_error(2), mean_vel_error(3));

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Error between velocities")
else
    figure2('name', "Error between velocities")
end
 
subplot(3,1,1)
hold on
grid on
plot(new_time, vel_error(:,1),'r')
ylabel("[m/s]")
title("Error in Vx")

subplot(3,1,2)
hold on
grid on
plot(new_time, vel_error(:,2),'r')
ylabel("[m/s]")
title("Error in Vy")

subplot(3,1,3)
hold on
grid on
plot(new_time, vel_error(:,3),'r')
ylabel("[m/s]")
title("Error in Vz")

