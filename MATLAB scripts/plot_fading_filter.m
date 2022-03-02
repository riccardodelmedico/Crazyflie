function plot_fading_filter(titleFigure, labelFigure, time, x1, x2, beta)
% x0= 0.08;
% y0= 0.07;
% width= 0.87;
% height= 0.9;
legendname= "Real";
hold on
grid on
plot(time, x1, 'r', 'LineWidth', 1.2)
len = length(beta);

for k = 1:1:len
    plot(time, x2(:,k))
    legendname(k+1)= sprintf('Filtered: beta = %0.2f',beta(k));
end

ylabel(labelFigure)
title(titleFigure,'Interpreter','latex')
legend(legendname)
% set(gca, 'Position', [x0,y0,width, height], 'FontSize', 15);
set(gcf, 'Color', 'w');