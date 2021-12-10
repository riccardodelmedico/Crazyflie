begin_date = cust_time(1);
end_date = cust_time(end);
new_time = [];
new_data_x = [];
new_data_y = [];

for i=1:size(int_time)
    if( isbetween(int_time(i), begin_date, end_date))
        new_time = [new_time; int_time(i)];
        new_data_x = [new_data_x; int_vx(i)];
        new_data_y = [new_data_y; int_vy(i)];
    end
end
int_drone_posx = interp1(cust_time, drone_vel_x, new_time);
int_drone_posy = interp1(cust_time, drone_vel_y, new_time);

error_x = interp1(new_time, new_data_x, new_time);
err_x = mean(abs(int_drone_posx - error_x))
error_y = interp1(new_time, new_data_y, new_time);
err_y = mean(abs(int_drone_posy - error_y))
