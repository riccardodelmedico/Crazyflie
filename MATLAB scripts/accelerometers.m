clear; close all; clc

%% File loading
name = "crazyfun__20220325_165854.txt";

current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);
vicon = fullfile(path, '..\vicon_data\', name);

delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
raw_vicon_data = importdata(vicon,delimiterIn,headerlinesIn);

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

clear vicon internal
clear raw_vicon_data raw_internal_data
clear current_file delimiterIn headerlinesIn name path

%% Data extraction
% Extracted data                        Variables meaning 
drone_posx = vicon_data(:,1);           % \
drone_posy = vicon_data(:,2);           %  |-> drone position from Vicon, in Vicon frame [m]
drone_posz = vicon_data(:,3);           % /
drone_quatx = vicon_data(:,4);          %  \
drone_quaty = vicon_data(:,5);          %   |-> drone orientation through quaternions from Vicon 
drone_quatz = vicon_data(:,6);          %  /
drone_quatw = vicon_data(:,7);          % /
cust_time = vicon_data(:,end);
% cust_time = cust_time - cust_time(1);

acc_x = internal_data(:,1);
acc_y = internal_data(:,2);
acc_z = internal_data(:,3);
int_roll = internal_data(:,4);          % \
int_pitch = -internal_data(:,5);        %  |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /
int_time = internal_data(:,end);
% int_time = int_time - int_time(1);

% acc_x = smooth(acc_x, 20);
% acc_y = smooth(acc_y, 20);
% acc_z = smooth(acc_z, 20);

% Conversion to Euler angles from the Vicon-generated quaternions
vicon_quat = [drone_quatw, drone_quatx, drone_quaty, drone_quatz];
vicon_euler = quat2eul(vicon_quat,'XYZ');

vicon_roll = interp1(cust_time, vicon_euler(:,1), int_time);
vicon_pitch = interp1(cust_time, vicon_euler(:,2), int_time);
vicon_yaw = interp1(cust_time, vicon_euler(:,3), int_time);

%% Acceleration plot
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
    % Internal Angles
    R_x = elem_rot(deg2rad(int_roll(i)), 1);
    R_y = elem_rot(deg2rad(int_pitch(i)), 2);
%     R_z = elem_rot(deg2rad(int_yaw(i)), 3);

    % Vicon Angles
%     R_x = elem_rot(vicon_roll(i), 1);
%     R_y = elem_rot(vicon_pitch(i), 2);

    %R = R_z * R_y * R_x; 
    R = R_y * R_x;
    %commandend_acc = R * comm_acc_b;

    world_acc(i,:) = R * acc(i,:)';
end

figure('name', 'x-acceleration in world frame')
title('X-acceleration with roll and pitch compensation', 'interpreter', 'latex')

hold on
grid on 
grid minor
plot(int_time,world_acc(:,1))
xlabel("$[s]$",'Interpreter', 'latex')
ylabel("$[m/s^2]$",'Interpreter', 'latex')
set(gca, 'FontSize', 18, 'Position', [0.06,0.1,0.92,0.86]);
set(gcf, 'Color', 'w');

figure('name', 'y-acceleration in world frame')
title('Y-acceleration with roll and pitch compensation', 'interpreter', 'latex')

hold on
grid on 
grid minor
plot(int_time,world_acc(:,2))
xlabel("$[s]$",'Interpreter', 'latex')
ylabel("$[m/s^2]$",'Interpreter', 'latex')
set(gca, 'FontSize', 18, 'Position', [0.06,0.1,0.92,0.86]);
set(gcf, 'Color', 'w');

figure('name', 'z-acceleration in world frame')
title('Z-acceleration with roll and pitch compensation', 'interpreter', 'latex')
hold on
grid on 
grid minor
plot(int_time,world_acc(:,3))
xlabel("$[s]$",'Interpreter', 'latex')
ylabel("$[m/s^2]$",'Interpreter', 'latex')
set(gca, 'FontSize', 18, 'Position', [0.06,0.1,0.92,0.86]);
set(gcf, 'Color', 'w');

