%% Test APNG con Wand Colpito 
%crazyfun__20220202_113447
%crazyfun__20220202_113721
%crazyfun__20220204_153006
%crazyfun__20220204_154443
%% Test APNG target virtuale 
%crazyfun__20220204_150714

%% Test PNG con Wand Colpito
%crazyfun__20220202_115346

%% Test PNG target virtuale colpito
%crazyfun__20220203_153354
%crazyfun__20220203_154452
%crazyfun__20220204_141955

%% Test PNG homing con Wand
%crazyfun__20220204_161237

%% Test APNG homing con Wand
%crazyfun__20220204_162022

%% Pollini intercettazione
%crazyfun__20220204_173513
%%
%clear; close all; clc

%% Homing flag
homing = 0;

%% File loading
name = "crazyfun__20220216_170028.txt";
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
target_acc_x = guidance_data(:, 12);
target_acc_y = guidance_data(:, 13);
est_yr_ddot = guidance_data(:, 14);
yr_ddot = guidance_data(:, 15);
est_yr_ddot_v2 = guidance_data(:,16);

if (homing == 1)
    intenal_yaw = guidance_data(:, 12);
    internal_yawrate = guidance_data(:, 13);
    est_yaw = guidance_data(:, 14);
    est_yawrate = guidance_data(:, 15);
end

command_yawrate = command_data(:,3);
command_time = datetime(command_data(:,end), 'ConvertFrom', 'datenum');

clear vicon_data internal_data

% %% Guidance Module
% % Pursuer-Target trajectories plot
% if exist('figure2') == 0  %#ok<*EXIST>
%     figure('name', "2D visualization in Vicon reference system")
% else
%     figure2('name', "2D visualization in Vicon reference system")
% end
% 
% hold on
% grid on
% xlabel("Vicon x axis [m]")
% ylabel("Vicon y axis [m]")
% axis equal 
% 
% plot(target_px, target_py, 'ko', ...
%      'MarkerSize', 8, 'MarkerFaceColor', 'y')
% plot(int_px, int_py, '-b')
% 
% legend("Target Position", "Internal Drone Position")
% title("Vicon reference system")

%% Commanded acceleration calculus
guidance_acc = -command_data(:,1).*command_data(:,3);

%% Animation of the guidance
% Interpolating the int data to fit them to guidance_time
figure('name', "Guidance animation, red=drone, yellow= target")
int_px_intepl= interp1(int_time, int_px, guidance_time);
int_py_intepl= interp1(int_time, int_py, guidance_time);
int_yaw_intepl= interp1(int_time, int_yaw, guidance_time);
hold on
grid on
xlabel("Vicon x axis [m]")
ylabel("Vicon y axis [m]")
axis equal 
for i= 1:1:length(command_time)
    plot(target_px(i), target_py(i), 'ko', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'y')
    plot(int_px_intepl(i), int_py_intepl(i), 'ko',...
     'MarkerSize', 8, 'MarkerFaceColor', 'r')
%     quiver(int_px_intepl(i), int_py_intepl(i), 0.01*guidance_acc(i)*cos(pi/2+ int_yaw_intepl(i)), 0.01*guidance_acc(i)*sin(pi/2+ int_yaw_intepl(i)))
    pause(0.02)
end
% %% Acceleration subsection 
% figure('name', 'X-Velocity of the target, comparison')
% hold on
% % axis equal
% 
% %plot(guidance_time(3:end), target_acc_x(3:end), 'r')
% vel_x = diff(target_px);
% vel_x = vel_x./0.02;
% sm_vel_x = smooth(vel_x,50);
% acc_x = diff(sm_vel_x);
% acc_x = acc_x./0.02;
% sm_acc_x = smooth(acc_x,50);
% plot(guidance_time(2:end),vel_x,'g--')
% plot(guidance_time(2:end),sm_vel_x,'b--')
% 
% figure('name', 'X-Acceleration of the target, comparison from diff of smooth and non-smooth')
% hold on
% nsm_acc_x= diff(vel_x);
% nsm_acc_x = nsm_acc_x./0.02;
% plot(guidance_time(3:end),acc_x,'g--')
% plot(guidance_time(3:end),sm_acc_x,'b--')
% plot(guidance_time(3:end),nsm_acc_x,'r--')
% 
% figure('name', 'Y-Acceleration of the target, comparison')
% hold on 
% vel_y = diff(target_py);
% vel_y = vel_y./0.02;
% vel_y = smooth(vel_y,50);
% acc_y = diff(vel_y);
% acc_y = acc_y./0.02;
% sm_acc_y = smooth(acc_y,50);
% plot(guidance_time(3:end),sm_acc_y,'b--')
% plot(guidance_time(3:end), target_acc_y(3:end), 'r')
% 
% figure('name', 'Target acceleration norm')
% for i= 1:1: length(guidance_time)
%     norm_i(i)= norm([target_acc_x(i);target_acc_y(i)],2);
% end
% plot(guidance_time, norm_i)
% norm=mean(norm_i(100:end));
% %% Range and Closing velocity plot: 
% %  comparison between closed form solution and fading filter
% if exist('figure2') == 0  %#ok<*EXIST>
%     figure('name', "Range(R) and Closing Velocity (Vc)")
% else
%     figure2('name', "Range(R) and Closing Velocity (Vc)")
% end
% 
% subplot(2,1,1)
% hold on
% grid on
% plot(guidance_time, guidance_r, 'r')
% plot(guidance_time, fading_r, 'b--')
% ylabel("[m]")
% title("Pursuer-Target Distance")
% 
% subplot(2,1,2)
% hold on
% grid on
% plot(guidance_time, -guidance_r_dot, 'r ')
% plot(guidance_time, -fading_r_dot, 'b--')
% ylabel("[m/s]")
% title("Closing Velocity")

%% LOS and LOS rate plot: 
%  comparison between closed form solution and fading filter
figure('name', "LOS (sigma) and LOS rate (sigma_dot)")
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

%% Range and Closing velocity

% if exist('figure2') == 0  %#ok<*EXIST>
%     figure('name', "Range(R) and Closing Velocity (Vc)")
% else
%     figure2('name', "Range(R) and Closing Velocity (Vc)")
% end
figure('name', "Range(R) and Closing Velocity (Vc)")
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
if(homing ==1)
    if exist('figure2') == 0  %#ok<*EXIST>
        figure('name', "Yaw and YawRate Plot ")
    else
        figure2('name', "Yaw and YawRate Plot")
    end

    subplot(2,1,1)
    hold on
    grid on
    plot(guidance_time, intenal_yaw, 'r ')
    plot(guidance_time, est_yaw, 'b--')
    ylabel("[rad]")
    title("Yaw")

    subplot(2,1,2)
    hold on
    grid on
    plot(guidance_time, internal_yawrate, 'r ')
    plot(guidance_time, est_yawrate, 'b--')
    ylabel("[rad/s]")
    title("YawRate")
end

%% Command time analysis
temporization = second(guidance_time);
period = diff(temporization);
figure('name', 'Temporization')
plot(period, 'b.')
mean_period = mean(period)

% %% Acceleration
% figure('name', 'yr_ddot vs est yr_ddot')
% hold on
% plot(guidance_time, est_yr_ddot, 'r')
% plot(guidance_time, yr_ddot, 'b')
% plot(guidance_time, est_yr_ddot_v2, 'g--')
% 

%% Commanded acceleration




