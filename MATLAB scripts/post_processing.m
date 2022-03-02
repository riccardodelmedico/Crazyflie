clear; clc; close all;

%% Data extraction
name = "crazyfun__20220225_101036.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

core = fullfile(path, '../core_data/', name);
guidance = fullfile(path, '../guidance_data/', name);
delimiterIn = ' ';
headerlinesIn = 1;
raw_core_data = importdata(core,delimiterIn,headerlinesIn);
raw_guidance_data = importdata(guidance,delimiterIn,headerlinesIn);
if isstruct(raw_core_data)
    core_data = raw_core_data.data;
else
    core_data = raw_core_data;
end
if isstruct(raw_guidance_data)
    guidance_data = raw_guidance_data.data;
else
    guidance_data = raw_guidance_data;
end

clear core
clear raw_core
clear current_file delimiterIn headerlinesIn name path

%% Data Extraction 
target_px = core_data(:,1);
target_py = core_data(:,2);
drone_px = core_data(:,3);
drone_py = core_data(:,4);
target_est_vx = core_data(:,5);
target_est_vy = core_data(:,6);
drone_est_vx = core_data(:,7);
drone_est_vy = core_data(:,8);
core_r = core_data(:,9);
core_sigma = core_data(:,10);
core_r_dot_ff = core_data(:,11);
core_sigma_dot_ff = core_data(:,12);
core_r_dot_kf = core_data(:,13);
core_sigma_dot_kf = core_data(:,14);
target_est_ax = core_data(:,16);
target_est_ay = core_data(:,17);
core_time = core_data(:,15);
drone_est_ax = core_data(:,18);
drone_est_ay = core_data(:,19);
core_r_kf = core_data(:,20);
core_sigma_kf = core_data(:,21);

guidance_r_dot = guidance_data(:,1);
guidance_sigma_dot = guidance_data(:,2);
guidance_time = guidance_data(:,3);


%% Smoothing Drone, Target and Guidance Measures (from Vicon)
% Drone
[drone_sm_vel_x, drone_sm_acc_x, drone_ir_vel_x, drone_ir_acc_x] = smoothing(drone_px, 20, core_time);
[drone_sm_vel_y, drone_sm_acc_y, drone_ir_vel_y, drone_ir_acc_y] = smoothing(drone_py, 20, core_time);
drone_vel = [drone_ir_vel_x, drone_sm_vel_x, drone_ir_vel_y, drone_sm_vel_y];
drone_acc = [drone_ir_acc_x, drone_sm_acc_x, drone_ir_acc_y, drone_sm_acc_y];
plot_smooth(0, core_time, drone_vel, drone_acc)

% Target
[target_sm_vel_x, target_sm_acc_x, target_ir_vel_x, target_ir_acc_x] = smoothing(target_px, 20, core_time);
[target_sm_vel_y, target_sm_acc_y, target_ir_vel_y, target_ir_acc_y] = smoothing(target_py, 20, core_time);
target_vel = [target_ir_vel_x, target_sm_vel_x, target_ir_vel_y, target_sm_vel_y];
target_acc = [target_ir_acc_x, target_sm_acc_x, target_ir_acc_y, target_sm_acc_y];
plot_smooth(1, core_time, target_vel, target_acc)

% Range and Sigma 
[sm_sigma_dot, ~, ir_sigma_dot, ~] = smoothing(core_sigma, 20, core_time);
[sm_r_dot, ~, ir_r_dot, ~] = smoothing(core_r, 20, core_time);
r_dot = [ir_r_dot, sm_r_dot];
sigma_dot = [ir_sigma_dot, sm_sigma_dot];
plot_smooth(2, core_time, r_dot, sigma_dot) 

%% Fading Filter Elaboration for Drone, Target and Guidance Measures
% Drone: probably the best result is between 0.45 and 0.5
beta_drone=[0.5,0.55,0.6];
[ff_drone_px, ff_drone_vx, ff_drone_ax] = FF(3, drone_px, 0, core_time, beta_drone);
[ff_drone_py, ff_drone_vy, ff_drone_ay] = FF(3, drone_py, 0, core_time, beta_drone);

figure('name', 'Comparison between smoothed and FF Drone measures')
subplot(2,2,1)
plot_fading_filter('Drone X-velocity from fading filter vs smoothed',...
         '[m/s]', core_time(2:end), drone_sm_vel_x, ff_drone_vx(2:end,:), beta_drone)
subplot(2,2,2)   
plot_fading_filter('Drone Y-velocity from fading filter vs smoothed',...
         '[m/s]', core_time(2:end), drone_sm_vel_y, ff_drone_vy(2:end,:), beta_drone)
subplot(2,2,3)
plot_fading_filter('Drone X-acceleration from fading filter vs smoothed',...
         '[m/s^2]', core_time(3:end), drone_sm_acc_x, ff_drone_ax(3:end,:), beta_drone)
subplot(2,2,4)
plot_fading_filter('Drone Y-acceleration from fading filter vs smoothed',...
         '[m/s^2]', core_time(3:end), drone_sm_acc_y, ff_drone_ay(3:end,:), beta_drone)

% Target: probably the best result is between 0.45 and 0.5
beta_target=[0.4,0.45,0.5];
[ff_target_px, ff_target_vx, ff_target_ax] = FF(3, target_px, 0, core_time, beta_target);
[ff_target_py, ff_target_vy, ff_target_ay] = FF(3, target_py, 0, core_time, beta_target);

figure('name', 'Comparison between smoothed and FF Target measures')
subplot(2,2,1)
plot_fading_filter('Target X-velocity from fading filter vs smoothed',...
         '[m/s]', core_time(2:end), target_sm_vel_x, ff_target_vx(2:end,:), beta_target)
subplot(2,2,2)     
plot_fading_filter('Target Y-velocity from fading filter vs smoothed',...
         '[m/s]', core_time(2:end), target_sm_vel_y, ff_target_vy(2:end,:), beta_target)
subplot(2,2,3)
plot_fading_filter('Target X-acceleration from fading filter vs smoothed',...
         '[m/s^2]', core_time(3:end), target_sm_acc_x, ff_target_ax(3:end,:), beta_target)
subplot(2,2,4)
plot_fading_filter('Target Y-acceleration from fading filter vs smoothed',...
         '[m/s^2]', core_time(3:end), target_sm_acc_y, ff_target_ay(3:end,:), beta_target)

% Guidance
beta_sigma = [0.3,0.35,0.4];
[ff_sigma, ff_sigma_dot, ~]= FF(2, core_sigma, sm_sigma_dot(1), core_time, beta_sigma);
beta_r = [0.3,0.4,0.5];
[ff_r, ff_r_dot, ~]= FF(2, core_r, sm_r_dot(1), core_time, beta_r);

figure('name', 'Comparison between smoothed and FF Guidance measures')
subplot(2,1,1)
plot_fading_filter('Smooth vs filtered $\dot{\sigma}$', '[rad/s]', core_time(2:end), sm_sigma_dot, ff_sigma_dot(2:end,:), beta_sigma);
subplot(2,1,2)
plot_fading_filter('Smooth vs filtered $\dot{R}$', '[m/s]', core_time(2:end), sm_r_dot, ff_r_dot(2:end,:), beta_r);



