clear; close all; clc

%% File loading
name = "crazyfun__20220311_161049.txt";
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

%% Data extraction
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

clear vicon_data internal_data
% x0= 0.08;
% y0= 0.07;
% width= 0.87;
% height= 0.9;

%% Animation of the guidance
figure('name', "2D Guidance Visualization in Vicon Reference Frame")
hold on
grid on
xlabel("Vicon x axis [m]",'Interpreter', 'latex')
ylabel("Vicon y axis [m]",'Interpreter', 'latex')
xlim([-1.5, 1.2])
ylim([-1.0, 2.0])
axis equal 
h = animatedline('Color', [1, 0, 0], 'LineWidth', 3);
f = animatedline('Color', [0, 0, 1], 'LineWidth', 3);

a = tic;
for i = 1:length(core_time)
    if(i == 1)                    
        target = plot(target_px(i),target_py(i), ...
						'o','Color',[0, 0, 0], 'MarkerSize', 8,...
						'MarkerFaceColor',[0, 0, 1.0]);
        drone = plot(drone_px(i),drone_py(i), ...
						'o','Color',[1.0, 0, 0], 'MarkerSize', 8,...
						'MarkerFaceColor',[1.0, 0, 0]);                    
    else
        drone.XData = drone_px(i);
        drone.YData = drone_py(i);
        target.XData = target_px(i);
        target.YData = target_py(i);
    end
    addpoints(h, drone_px(i), drone_py(i));
    addpoints(f, target_px(i), target_py(i));
    drawnow limitrate 
    pause(0.01)
end

%% LOS and LOS rate plot
figure('name', "LOS (sigma) and LOS rate (sigma_dot)")
subplot(2,1,1)
hold on
grid on
plot(core_time, core_sigma, 'b--')
plot(core_time, core_sigma_kf, 'g')
xlabel("[s]",'Interpreter', 'latex')
ylabel("[rad]",'Interpreter', 'latex')
title("LOS angle",'Interpreter', 'latex')
legend('$\sigma$ computed with only Vicon measures', '$\sigma$ computed with mixed (KF and Vicon) measures', ...
        'Interpreter', 'latex')

        
subplot(2,1,2)
hold on
grid on
plot(guidance_time, guidance_sigma_dot, 'r ')
plot(core_time, core_sigma_dot_ff, 'b--')
plot(core_time, core_sigma_dot_kf, 'g')
xlabel("[s]",'Interpreter', 'latex')
ylabel("[rad/s]",'Interpreter', 'latex')
title("LOS angle derivative",'Interpreter', 'latex')
legend('$\dot{\sigma}$ computed on board with FF',...
       '$\dot{\sigma}$ computed with closed-form equation using only Vicon', ...
       '$\dot{\sigma}$ computed with closed-form equation using Vicon and KF', ...
        'Interpreter', 'latex')
% set(gca, 'Position', [x0,y0,width, height], 'FontSize', 15);
set(gcf, 'Color', 'w');

    
%% Range and Closing Velocity plot
figure('name', "Range(R) and Closing Velocity(-R_dot)")
subplot(2,1,1)
hold on
grid on
plot(core_time, core_r, 'b--')
plot(core_time, core_r_kf, 'g')
xlabel("[s]",'Interpreter', 'latex')
ylabel("[m]",'Interpreter', 'latex')
title("Range",'Interpreter', 'latex')
legend('$R$ computed with only Vicon measures', '$R$ computed with mixed (KF and Vicon) measures', ...
        'Interpreter', 'latex')
        
subplot(2,1,2)
hold on
grid on
plot(guidance_time, -guidance_r_dot, 'r ')
plot(core_time, -core_r_dot_ff, 'b--')
plot(core_time, -core_r_dot_kf, 'g')
xlabel("[s]",'Interpreter', 'latex')
ylabel("[m/s]",'Interpreter', 'latex')
title("Closing Velocity",'Interpreter', 'latex')
legend('$V_c$ computed on board with FF',...
       '$V_c$ computed with closed-form equation using only Vicon', ...
       '$V_c$ computed with closed-form equation using Vicon and KF', ...
        'Interpreter', 'latex')
    
%% Acceleration subsection 



%% Timing analysis
figure('name', "Threads Timing")
subplot(2,1,1)
hold on
grid on
xlabel("[s]",'Interpreter', 'latex')
ylabel("[s]",'Interpreter', 'latex')
dt_core = diff(core_time);
title('Timing of DataCore Thread','Interpreter', 'latex')
plot(core_time(2:end), dt_core, 'b.')
mean_dt_core = mean(dt_core);
legend_dc = sprintf('Mean dt: %0.5f',mean_dt_core);
legend(legend_dc)

subplot(2,1,2)
hold on
grid on
xlabel("[s]",'Interpreter', 'latex')
ylabel("[s]",'Interpreter', 'latex')
dt_guidance = diff(guidance_time);
title('Timing of DroneGuidance Thread','Interpreter', 'latex')
plot(guidance_time(2:end), dt_guidance, 'b.')
mean_dt_guidance = mean(dt_guidance);
legend_g = sprintf('Mean dt: %0.5f',mean_dt_guidance);
legend(legend_g)
