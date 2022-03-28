clear; close all; clc

%% File loading
name = "crazyfun__20220325_165854.txt";

current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);


delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
if isstruct(raw_internal_data)
    internal_data = raw_internal_data.data;
else
    internal_data = raw_internal_data;
end


clear wand vicon setpoints internal
clear raw_set_data raw_vicon_data raw_wand raw_internal_data
clear current_file delimiterIn headerlinesIn name path

%% set acceleration
acc_x = internal_data(:,1);
acc_y = internal_data(:,2);
acc_z = internal_data(:,3);
int_roll = internal_data(:,4);          % \
int_pitch = -internal_data(:,5);        %  |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /
int_time = internal_data(:,end);
int_time = int_time - int_time(1);

% acc_x = smooth(acc_x, 20);
% acc_y = smooth(acc_y, 20);
% acc_z = smooth(acc_z, 20);

%% plot acc
figure('name', 'x-acceleration in body frame')
hold on
grid on
grid minor
plot(int_time,acc_x)

figure('name', 'y-acceleration in body frame')
hold on
grid on
grid minor
plot(int_time,acc_y)

figure('name', 'z-acceleration in body frame')
hold on
grid on
grid minor
plot(int_time,acc_z)
acc = [acc_x, acc_y, acc_z];
world_acc= zeros(length(int_time),3);
comm_acc_b = [0; -0.5; 0];

for i=1:1:length(int_time)
    R_x = elem_rot(deg2rad(int_roll(i)), 1);
    R_y = elem_rot(deg2rad(int_pitch(i)), 2);
    R_z = elem_rot(deg2rad(int_yaw(i)), 3); 
    %R = R_z * R_y * R_x; 
    R = R_y * R_x;
    %commandend_acc = R * comm_acc_b;

    world_acc(i,:) = R * acc(i,:)';
end

% world_acc(:,1) = smooth(world_acc(:,1), 20);
% world_acc(:,2) = smooth(world_acc(:,2), 20);
% world_acc(:,3) = smooth(world_acc(:,3), 20);

figure('name', 'x-acceleration in world frame')
hold on
grid on 
grid minor
plot(int_time,world_acc(:,1))

figure('name', 'y-acceleration in world frame')
hold on
grid on 
grid minor
plot(int_time,world_acc(:,2))

figure('name', 'z-acceleration in world frame')
hold on
grid on 
grid minor
plot(int_time,world_acc(:,3))

