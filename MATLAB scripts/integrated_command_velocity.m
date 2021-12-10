function int_com_vel = integrated_command_velocity(command_vel,initial_pos)
int_com_vel= zeros(length(command_vel),1);
dt=0.1;
    for i= 1:length(command_vel)
        if(i==1)
            int_com_vel(i)=initial_pos;
        else
            int_com_vel(i)=int_com_vel(i-1)+command_vel(i)*dt;
        end
    end
end

