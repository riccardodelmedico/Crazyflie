function plot_fading_filter(titleFigure, labelFigure, time, x1, x2, beta)

legendname= "Real";
hold on
grid on
plot(time, x1, 'r', 'LineWidth', 1.2)
len = length(beta);

for k = 1:1:len
    plot(time, x2(:,k))
    legendname(k+1)= sprintf('Filtered: beta = %0.2f',beta(k));
    set(gca, 'FontSize', 17);
end

ylabel(labelFigure,'interpreter','latex')
xlabel('$[s]$','interpreter','latex')
title(titleFigure,'Interpreter','latex')
legend(legendname,'interpreter','latex')

set(gcf, 'Color', 'w');