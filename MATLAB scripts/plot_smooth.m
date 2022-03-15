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
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[m/s]$",'interpreter','latex')
    title_1 = sprintf('%s x-velocity', var); 
    title(title_1,'interpreter','latex')
    plot(time(2:end), x1(:, 1), 'r')
    plot(time(2:end), x1(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);
    
    subplot(2,2,2)
    hold on
    grid on
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[m/s]$",'interpreter','latex')
    title_2 = sprintf('%s y-velocity', var); 
    title(title_2,'interpreter','latex')
    plot(time(2:end), x1(:, 3), 'r')
    plot(time(2:end), x1(:, 4), 'b')
    legend('Incremental ratio', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);
    
    subplot(2,2,3)
    hold on
    grid on
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[m/s^2]$",'interpreter','latex')
    title_3 = sprintf('%s x-acceleration', var); 
    title(title_3,'interpreter','latex')
    plot(time(3:end), x2(:, 1), 'r')
    plot(time(3:end), x2(:, 2), 'b')
    legend('Incremental ratio (of smoothed x-velocity)', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);
    
    subplot(2,2,4)
    hold on
    grid on
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[m/s^2]$",'interpreter','latex')
    title_4 = sprintf('%s y-acceleration', var); 
    title(title_4,'interpreter','latex')
    plot(time(3:end), x2(:, 3), 'r')
    plot(time(3:end), x2(:, 4), 'b')
    legend('Incremental ratio (of smoothed y-velocity)', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);

else
    figure('name', 'Comparison between range and LOS rate smoothed and not')
    subplot(2,1,1)
    hold on
    grid on
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[m/s]$",'interpreter','latex')
    title("Derivative of Range",'interpreter','latex')
    plot(time(2:end), x1(:, 1), 'r')
    plot(time(2:end), x1(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);
    
    subplot(2,1,2)
    hold on
    grid on
    xlabel("$[s]$",'interpreter','latex')
    ylabel("$[rad/s]$",'interpreter','latex')
    title("Derivative of LOS",'interpreter','latex')
    plot(time(2:end), x2(:, 1), 'r')
    plot(time(2:end), x2(:, 2), 'b') 
    legend('Incremental ratio', 'Smoothed','interpreter','latex')
    set(gca, 'FontSize', 17);



end
end



