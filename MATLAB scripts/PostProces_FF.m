
%% Load Data

name = "crazyfun__20220207_161722.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);
[path, ~, ~] = fileparts(path);
internal = fullfile(path, '/internal_data/', name);
guidance = fullfile(path, '/guidance_data/', name);
command = fullfile(path, '/command_data/', name);
delimiterIn = ' ';
headerlinesIn = 1;

raw_guidance_data = importdata(guidance,delimiterIn,headerlinesIn);

if isstruct(raw_guidance_data)
    guidance_data = raw_guidance_data.data;
else
    guidance_data = raw_guidance_data;
end



clear guidance 
clear raw_guidance
clear current_file delimiterIn headerlinesIn name path
%% Choose Guidance 

target_px = guidance_data(:,1);
target_py = guidance_data(:,2);
guidance_r = guidance_data(:,3);
guidance_vc = guidance_data(:,4);
guidance_sigma_dot = guidance_data(:,5);
guidance_sigma = guidance_data(:, 6);
guidance_time = datetime(guidance_data(:,end), 'ConvertFrom', 'datenum');
ini =second( guidance_time(1));
size_vec = size(guidance_time);

%% Plot Sigma Dot


%% Fading Filter Variable

beta = 0.7;
G = 1 -beta^3;
H = 1.5*((1-beta)^2)*(1+beta);
K = 0.5 *(1-beta)^3;
dt = (second(guidance_time(end)) - second(guidance_time(1)))/size_vec(1);
s_est = guidance_sigma(1);
s_dot_est =   guidance_sigma_dot(1);
s_ddot_est = 0;
list_s_dot_est = s_dot_est;
list_RI= [s_dot_est];
error = 0;
for i=2:size_vec(1)
%     dt = second( guidance_time(i))-second(guidance_time(i-1));
    new_s_est = s_est + s_dot_est * dt + 0.5* s_ddot_est *dt^2;
    s_est = new_s_est + G* (guidance_sigma(i)-new_s_est);
    list_s_est(i) = s_est;
    
    s_dot_est = s_dot_est+ dt * s_ddot_est + (H/dt)* (guidance_sigma(i)-new_s_est);
%     s_dot_RI = (list_s_dot_est(end)-s_dot_est)/dt;
%     list_RI(i) =  (s_dot_RI);
    list_s_dot_est(i) = (s_dot_est);
    s_ddot_est = s_ddot_est + ((2*K)/dt^2)*(guidance_sigma(i)-new_s_est);
    list_s_ddot_est(i) = s_ddot_est;
    error(i) = guidance_sigma_dot(i)- s_dot_est; 
    
end

% lin_space = 
%% Plot Stime e Variabili Reali
% hold on 
% grid on 
% plot (guidance_time,error,'r')
% % plot(guidance_time,list_RI,'g--')
% 
% % hold off
% 
% hold on 
% plot(guidance_time,guidance_sigma)
% plot (guidance_time,list_s_est,'b--')
% hold off
d_s = gradient(guidance_sigma_dot(1:end-2),dt);
figure(1)
hold on 
plot(guidance_time,guidance_sigma_dot,'r')
plot (guidance_time,list_s_dot_est,'b--')

s_dot_smooth = smooth(d_s);
figure(2)
hold on 
plot(guidance_time(1:end-2),s_dot_smooth,'r')
plot (guidance_time(1:end-2),list_s_ddot_est(1:end-2),'b--')

% figure(3)
% hold on 
% plot(guidance_time(1:end-2),guidance_sigma_dot,'r*')
% plot(guidance_time,,'b*-')


