clear; close all; clc

%% File loading
name = "crazyfun__20220224_110554.txt";
% crazyfun__20220124_121109, crazyfun__20220124_140955 test con correzioni e parametri default senza anelli 
% crazyfun__20220124_121350 con correzioni di angolo e parametri di kalman
% nostri
% crazyfun__20220124_140634 test senza correzioni e con parametri di
% default 
% crazyfun__20220124_121702 definitivo

% crazyfun__20220124_122329 test di rotazione 360 gradi con correzioni di
% angolo e parametri di kalman nostri---> si schianta
% crazyfun__20220124_122522 test di rotazione 360 gradi senza correzioni 

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
 
clear wand vicon setpoints internal
clear raw_set_data raw_vicon_data raw_wand raw_internal_data
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
cust_time = cust_time - cust_time(1);


int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_pz = internal_data(:,3);            % /
int_roll = internal_data(:,4);          % \
int_pitch = -internal_data(:,5);        %  |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /
int_time = internal_data(:,end);
int_time = int_time - int_time(1);

clear wand_data set_data vicon_data internal_data
%% Analysis
% MATLAB uses q = [w x y z]
% Vicon creates q = [x y z w]

% Conversion to Euler angles from the Vicon-generated quaternions
vicon_quat = [drone_quatw, drone_quatx, drone_quaty, drone_quatz];
vicon_euler = rad2deg(quat2eul(vicon_quat,'XYZ'));

% orientation quaternion derived from internal estimate
crazy_euler = [int_roll, int_pitch, int_yaw];

% clear vicon_quat drone_quatw drone_quatx drone_quaty drone_quatz
%% Comparison between Euler angles
% This section analyzes the drone's internal attitude estimation with the
% Vicon-captured angles

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Comparison between Euler angles")
else
    figure2('name', "Comparison between Euler angles")
end

subplot(3,1,1)
hold on
grid on
plot(cust_time, vicon_euler(:,1),'r')
plot(int_time, crazy_euler(:,1),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Roll")

subplot(3,1,2)
hold on
grid on
plot(cust_time, vicon_euler(:,2),'r')
plot(int_time, crazy_euler(:,2),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Pitch")

subplot(3,1,3)
hold on
grid on
plot(cust_time, vicon_euler(:,3),'r')
plot(int_time, crazy_euler(:,3),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Yaw")

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
plot(cust_time, drone_posx,'r')
plot(int_time, int_px,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_posy,'r')
plot(int_time, int_py,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_posz,'r')
plot(int_time, int_pz,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Z coordinates")

%% Error between internal and vicon data
% This section analyzes the estimate difference between internal data (from
% EKF) and vicon data 

%interpolated vicon data with removal of NaN elements
interpolated_vicon_x = interp1(cust_time, drone_posx, int_time);
interpolated_vicon_x = rmmissing(interpolated_vicon_x);
interpolated_vicon_y = interp1(cust_time, drone_posy, int_time);
interpolated_vicon_y = rmmissing(interpolated_vicon_y);
interpolated_vicon_z = interp1(cust_time, drone_posz, int_time);
interpolated_vicon_z = rmmissing(interpolated_vicon_z);

int_px = int_px(1:length(interpolated_vicon_x));
error_x = abs(interpolated_vicon_x - int_px);
mean_error_x = mean(error_x);

int_py = int_py(1:length(interpolated_vicon_y));
error_y = abs(interpolated_vicon_y - int_py);
mean_error_y = mean(error_y);

int_pz = int_pz(1:length(interpolated_vicon_z));
error_z = abs(interpolated_vicon_z - int_pz);
mean_error_z = mean(error_z);

fprintf("Mean Position Error:\n x: %f, y: %f, z: %f\n", ...
        mean_error_x, mean_error_y, mean_error_z);

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Error between positions")
else
    figure2('name', "Error between positions")
end

subplot(3,1,1)
hold on
grid on
plot(int_time(1:length(interpolated_vicon_x)), error_x,'r')
ylabel("meter [m]")
title("Error in X coordinates")

subplot(3,1,2)
hold on
grid on
plot(int_time(1:length(interpolated_vicon_y)), error_y,'r')
ylabel("meter [m]")
title("Error in Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(int_time(1:length(interpolated_vicon_z)), error_z,'r')
ylabel("meter [m]")
title("Error in Z coordinates")
