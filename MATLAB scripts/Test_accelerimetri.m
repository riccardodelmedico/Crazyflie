clear; close all; clc

%% File loading
name = "crazyfun__20220325_170255.txt";
% crazyfun__20220315_110052 : default parameters with angles correction
% crazyfun__20220315_110420 : modified parameters with angles correction
% crazyfun__20220315_110817 : modified parameters without angles correction
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

%% plot acc
figure(1)
grid on
grid minor
plot(int_time,acc_x)

figure(2)
grid on
grid minor
plot(int_time,acc_y)

figure(3)
grid on
grid minor
plot(int_time,acc_z)
acc = [acc_x, acc_y, acc_z];
world_acc= zeros(length(int_time),3);
for i=1:1:length(int_time)
    R_x = elem_rot(int_roll(i),1);
    R_y = elem_rot(- int_pitch(i),2);
    R_z = elem_rot(int_yaw(i),3); 
    R = R_z * R_y * R_x; 
        
    world_acc(i,:) = R * acc(i,:)';
end

figure(4)
grid on 
plot(int_time,world_acc(:,1))

figure(5)
grid on 
plot(int_time,world_acc(:,2))

figure(6)
grid on 
plot(int_time,world_acc(:,3))

