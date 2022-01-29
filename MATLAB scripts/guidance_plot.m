clear; close all; clc

%% File loading
name = "crazyfun__20220128_100815.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '../internal_data/', name);
guidance = fullfile(path, '..\guidance_data\', name);
command = fullfile(path, '..\command_data\', name);
delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
raw_guidance_data = importdata(guidance,delimiterIn,headerlinesIn);
raw_command_data = importdata(command,delimiterIn,headerlinesIn);

if isstruct(raw_internal_data)
    internal_data = raw_internal_data.data;
else
    internal_data = raw_internal_data;
end

if isstruct(raw_guidance_data)
    guidance_data = raw_guidance_data.data;
else
    guidance_data = raw_guidance_data;
end

if isstruct(raw_command_data)
    command_data = raw_command_data.data;
else
    command_data = raw_command_data;
end

clear guidance internal command 
clear raw_guidance_data raw_internal_data raw_command_data
clear current_file delimiterIn headerlinesIn name path

%% Data extraction
int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_yaw = internal_data(:,3);           % /
int_yawrate = internal_data(:,6);
int_time = datetime(internal_data(:,end), 'ConvertFrom', 'datenum');

target_px = guidance_data(:,1);
target_py = guidance_data(:,2);
guidance_r = guidance_data(:,3);
guidance_sigma = guidance_data(:, 4);
guidance_r_dot = guidance_data(:, 5);
guidance_sigma_dot = guidance_data(:, 6);
fading_r = guidance_data(:, 7);
fading_sigma = guidance_data(:, 8);
fading_r_dot = guidance_data(:, 9);
fading_sigma_dot = guidance_data(:, 10);
guidance_time = datetime(guidance_data(:, 11), 'ConvertFrom', 'datenum');

command_yawrate = command_data(:,3);
command_time = datetime(command_data(:,end), 'ConvertFrom', 'datenum');

clear vicon_data internal_data

%% Guidance Module
% Pursuer-Target trajectories plot
if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "2D visualization in Vicon reference system")
else
    figure2('name', "2D visualization in Vicon reference system")
end

hold on
grid on
xlabel("Vicon x axis [m]")
ylabel("Vicon y axis [m]")
axis equal 

plot(target_px, target_py, 'ko', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'y')
plot(int_px, int_py, '-b')

legend("Target Position", "Internal Drone Position")
title("Vicon reference system")

%% Range and Closing velocity plot: 
%  comparison between closed form solution and fading filter
if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Range(R) and Closing Velocity (Vc)")
else
    figure2('name', "Range(R) and Closing Velocity (Vc)")
end

subplot(2,1,1)
hold on
grid on
plot(guidance_time, guidance_r, 'r')
plot(guidance_time, fading_r, 'b--')
ylabel("[m]")
title("Pursuer-Target Distance")

subplot(2,1,2)
hold on
grid on
plot(guidance_time, -guidance_r_dot, 'r ')
plot(guidance_time, -fading_r_dot, 'b--')
ylabel("[m/s]")
title("Closing Velocity")

%% LOS and LOS rate plot: 
%  comparison between closed form solution and fading filter
if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "LOS (sigma) and LOS rate (sigma_dot)")
else
    figure2('name', "LOS (sigma) and LOS rate (sigma_dot)")
end

subplot(2,1,1)
hold on
grid on
plot(guidance_time, guidance_sigma, 'r ')
plot(guidance_time, fading_sigma, 'b--')
ylabel("[rad]")
title("LOS angle")

subplot(2,1,2)
hold on
grid on
plot(guidance_time, guidance_sigma_dot, 'r ')
plot(guidance_time, fading_sigma_dot, 'b--')
ylabel("[rad/s]")
title("LOS angle derivative")







