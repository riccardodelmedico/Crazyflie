%clear; clc; close all;

%% Data extraction
name = "crazyfun__20220216_170028.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

guidance = fullfile(path, '../guidance_data/', name);
delimiterIn = ' ';
headerlinesIn = 1;
raw_guidance_data = importdata(guidance,delimiterIn,headerlinesIn);
if isstruct(raw_guidance_data)
    guidance_data = raw_guidance_data.data;
else
    guidance_data = raw_guidance_data;
end

clear current_file delimiterIn headerlinesIn name path

%% Data Extraction 
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
guidance_time = guidance_time - guidance_time(1);
guidance_time = seconds(guidance_time);
target_acc_x = guidance_data(:, 12);
target_acc_y = guidance_data(:, 13);
est_yr_ddot = guidance_data(:, 14);
yr_ddot = guidance_data(:, 15);
est_yr_ddot_v2 = guidance_data(:,16);

%% Smoothing Drone, Target and Guidance Measures (from Vicon)
% Target
[target_sm_vel_x, target_sm_acc_x, target_ir_vel_x, target_ir_acc_x] = smoothing(target_px, 20, guidance_time);
[target_sm_vel_y, target_sm_acc_y, target_ir_vel_y, target_ir_acc_y] = smoothing(target_py, 20, guidance_time);
target_vel = [target_ir_vel_x, target_sm_vel_x, target_ir_vel_y, target_sm_vel_y];
target_acc = [target_ir_acc_x, target_sm_acc_x, target_ir_acc_y, target_sm_acc_y];
plot_smooth(1, guidance_time, target_vel, target_acc)

% Range and Sigma 
[sm_sigma_dot, ~, ir_sigma_dot, ~] = smoothing(guidance_sigma, 20, guidance_time);
[sm_r_dot, ~, ir_r_dot, ~] = smoothing(guidance_r, 20, guidance_time);
r_dot = [ir_r_dot, sm_r_dot];
sigma_dot = [ir_sigma_dot, sm_sigma_dot];
plot_smooth(2, guidance_time, r_dot, sigma_dot) 
