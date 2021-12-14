function [error, mean_error] = get_error(cust_time, new_time, ...
                                         vicon_quantity, new_quantity)
%GET_ERROR ritorna il vettore 'error' calcolato come la differenza passo
% passo tra vicon_quantity e new_quantity. Per questo calcolo è necessario
% adattare le loro dimensioni in modo da essere coerenti. new_time è
% l'intervallo di tempo in cui ci sono misure sia interne che del vicon, 
% mentre new_quantity sono le misure interne ottenute in tale intervallo. 
% Si è scelto di interpolare i dati vicon (che sono mandati a frequenza
% minore) lungo new_time

%interpolated vicon data
interpolated_vicon_x = interp1(cust_time, vicon_quantity(:,1), new_time);
interpolated_vicon_y = interp1(cust_time, vicon_quantity(:,2), new_time);
interpolated_vicon_z = interp1(cust_time, vicon_quantity(:,3), new_time);

% x-error
new_x = interp1(new_time, new_quantity(:,1), new_time);
error_x = abs(interpolated_vicon_x - new_x);
mean_error_x = mean(error_x);

% y-error
new_y = interp1(new_time, new_quantity(:,2), new_time);
error_y = abs(interpolated_vicon_y - new_y);
mean_error_y = mean(error_y);

% z-velocity
new_z = interp1(new_time, new_quantity(:,3), new_time);
error_z = abs(interpolated_vicon_z - new_z);
mean_error_z = mean(error_z);

error = [error_x, error_y, error_z];
mean_error = [mean_error_x, mean_error_y, mean_error_z];

end

