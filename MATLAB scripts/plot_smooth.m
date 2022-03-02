function plot_smooth(object, time, x1, x2)
if(object == 0 || object == 1)
    if(object == 0)
        var = "Drone";
    else 
        var = "Target";
    end
    
    name = sprintf('Comparison between %s velocity and acceleration smoothed and not', var);
    figure('name', name)
    subplot(2,2,1)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[m/s]")
    title_1 = sprintf('%s x-velocity', var); 
    title(title_1)
    plot(time(2:end), x1(:, 1), 'r')
    plot(time(2:end), x1(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed')
    
    subplot(2,2,2)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[m/s]")
    title_2 = sprintf('%s y-velocity', var); 
    title(title_2)
    plot(time(2:end), x1(:, 3), 'r')
    plot(time(2:end), x1(:, 4), 'b')
    legend('Incremental ratio', 'Smoothed')
    
    subplot(2,2,3)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[m/s^2]")
    title_3 = sprintf('%s x-acceleration', var); 
    title(title_3)
    plot(time(3:end), x2(:, 1), 'r')
    plot(time(3:end), x2(:, 2), 'b')
    legend('Incremental ratio (of smoothed x-velocity)', 'Smoothed')
    
    subplot(2,2,4)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[m/s^2]")
    title_4 = sprintf('%s y-acceleration', var); 
    title(title_4)
    plot(time(3:end), x2(:, 3), 'r')
    plot(time(3:end), x2(:, 4), 'b')
    legend('Incremental ratio (of smoothed y-velocity)', 'Smoothed')
else
    figure('name', 'Comparison between range and LOS rate smoothed and not')
    subplot(2,1,1)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[m/s]")
    title("Derivative of Range")
    plot(time(2:end), x1(:, 1), 'r')
    plot(time(2:end), x1(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed')
    
    subplot(2,1,2)
    hold on
    grid on
    xlabel("[s]")
    ylabel("[rad/s]")
    title("Derivative of LOS")
    plot(time(2:end), x2(:, 1), 'r')
    plot(time(2:end), x2(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed')
    
end    
end

