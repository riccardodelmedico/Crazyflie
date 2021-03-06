clear; close all; clc

%% File loading
%test for validation 
%crazyfun__20220111_144209 Step [0,120] [degree/s] time 10 [s]
% %crazyfun__20220111_144634 Ramp [36][degree/s^2] time 10[s]
%"crazyfun__20220111_122640" [0,1] Hz/ Ampl 30?[degree] time 10 [s]

name = "crazyfun__20220111_122640.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);
command = fullfile(path, '..\command_data\', name);
delimiterIn = ' ';
headerlinesIn = 1;
raw_internal_data = importdata(internal,delimiterIn,headerlinesIn);
raw_command_data = importdata(command,delimiterIn,headerlinesIn);

if isstruct(raw_internal_data)
    internal_data = raw_internal_data.data;
else
    internal_data = raw_internal_data;
end

if isstruct(raw_command_data)
    command_data = raw_command_data.data;
else
    command_data = raw_command_data;
end

clear internal command 
clear raw_internal_data raw_command_data
clear current_file delimiterIn headerlinesIn name path

%% Data extraction and append
new_int_yawrate = internal_data(:,6);
new_int_time = datetime(internal_data(:,end), 'ConvertFrom', 'datenum');

new_command_yawrate = command_data(:,3);
new_command_time = datetime(command_data(:,end), 'ConvertFrom', 'datenum');

clear internal_data command_data

% conversion from milliradians/s to degrees/s and interpolation of internal
% data in order to have them consistent with command_data time
new_int_yawrate = interp1(new_int_time, -new_int_yawrate*(18/(pi*100)), new_command_time);

if(exist('val_identification_data.mat', 'file') == 0)
    val_int_yawrate = new_int_yawrate;
    val_command_yawrate = new_command_yawrate;
    val_command_time = new_command_time;
else
    load('val_identification_data', 'val_int_yawrate', 'val_command_yawrate', 'val_command_time');
    val_int_yawrate = [val_int_yawrate; new_int_yawrate];
    val_command_yawrate = [val_command_yawrate; new_command_yawrate];
    diff = between(val_command_time(end),new_command_time(1),'Time');
    new_command_time = new_command_time - diff;
    val_command_time = [val_command_time; new_command_time];
end

save('val_identification_data', 'val_int_yawrate', 'val_command_yawrate', 'val_command_time');

%% Visualizing data
hold on; grid on
plot(val_command_time, val_command_yawrate)
plot(val_command_time, val_int_yawrate)
%%
