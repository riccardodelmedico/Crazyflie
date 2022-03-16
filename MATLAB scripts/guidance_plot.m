clear; close all; clc

% tempi dati da time.time() => test buoni:

% crazyfun__20220207_161441 (targer accelerante)
% crazyfun__20220207_161722 (targer accelerante)
% crazyfun__20220209_151202 (target vel. costante)
% crazyfun__20220209_151726 (target accelerante)
% crazyfun__20220209_154441 (target accelerante)

% tempi dati da Vicon => test buoni:
% crazyfun__20220216_145025 (vicon ma utilizzando sempre time.time() )
% crazyfun__20220216_165228 (target acc.)
% crazyfun__20220216_170028 (target vel. costante) 
%% File loading
name = "crazyfun__20220216_165228.txt";
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
% yr_ddot = guidance_data(:, 15);
% est_yr_ddot_v2 = guidance_data(:,16);

command_yawrate = command_data(:,3);
command_time = datetime(command_data(:,end), 'ConvertFrom', 'datenum');

clear vicon_data internal_data

%% Animation of the guidance
figure('name', "2D Guidance Visualization in Vicon Reference Frame")
int_px_intepl= interp1(int_time, int_px, command_time);
int_py_intepl= interp1(int_time, int_py, command_time);
int_yaw_intepl= interp1(int_time, int_yaw, guidance_time);
hold on
grid on
xlabel("Vicon x axis [m]",'Interpreter', 'latex')
ylabel("Vicon y axis [m]",'Interpreter', 'latex')
xlim([-1.5, 1.2])
ylim([-1.5, 2.0])
axis equal 
h = animatedline('Color', [1, 0, 0], 'LineWidth', 3);
f = animatedline('Color', [0, 0, 1], 'LineWidth', 3);

for i = 1:length(command_time)
    if(i == 1)                    
        target = plot(target_px(i),target_py(i), ...
						'o','Color',[0, 0, 0], 'MarkerSize', 8,...
						'MarkerFaceColor',[0, 0, 1.0]);
        drone = plot(int_px_intepl(i), int_py_intepl(i), ...
						'o','Color',[1.0, 0, 0], 'MarkerSize', 8,...
						'MarkerFaceColor',[1.0, 0, 0]);                    
    else
        drone.XData = int_px_intepl(i);
        drone.YData = int_py_intepl(i);
        target.XData = target_px(i);
        target.YData = target_py(i);
    end
    addpoints(h, int_px_intepl(i), int_py_intepl(i));
    addpoints(f, target_px(i), target_py(i));
    drawnow limitrate 
    pause(0.01)
end
set(gcf, 'Color', 'w');

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

%% Range and Closing velocity
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

%% Timing analysis
temporization = second(guidance_time);
temporization = temporization - temporization(1);
period = diff(temporization);
figure('name', 'Timing Analysis')
plot(temporization(2:end), period, 'b.')
title('Timing Analysis', 'Interpreter','latex')
xlabel("$[s]$", 'Interpreter','latex')
ylabel("$[s]$", 'Interpreter','latex')
set(gca, 'Fontsize', 15)
set(gcf, 'Color', 'w');
mean_period = mean(period)
