function [x_est, x_dot_est, x_ddot_est] = FF(order, x, x_dot, time, beta)
%FF function implements fading filter equations
%   User can decide the order of the filter and the beta parameter; time is
%   the time vector from which dt's are obtained;
%   x: measures vector  
%   x_dot: initial first derivative estimation
n = length(x);
nb = length(beta);
x_est = zeros(n,nb);
x_dot_est = zeros(n,nb);
x_ddot_est = zeros(n,nb);
delta_t = (diff(time));
for k=1:1:nb
    if order == 2
        G = 1 - beta(k)^2;
        H = (1 - beta(k))^2;
        x_est(1,k) = x(1);
        x_dot_est(1,k) = x_dot;
        for i = 2:1:n   
            tmp = x_est(i-1,k) + x_dot_est(i-1,k) * delta_t(i-1); 
            x_est(i,k) = tmp + G * (x(i) - tmp);
            x_dot_est(i,k) = x_dot_est(i-1,k) + H/delta_t(i-1) * (x(i) - tmp);
        end
    end

    if order == 3
        G = 1 - beta(k)^3;
        H = 1.5*(1 - beta(k))^2 * (1 + beta(k));
        K = 0.5*(1 - beta(k))^3;
        x_est(1,k) = x(1);
        x_dot_est(1,k) = x_dot;
        x_ddot_est(1,k) = 0;
        for i = 2:1:n
            tmp = x_est(i-1,k) + x_dot_est(i-1,k) * delta_t(i-1) + 0.5 * x_ddot_est(i-1,k) * delta_t(i-1)^2;
            x_est(i,k) = tmp + G * (x(i) - tmp);
            x_dot_est(i,k) = x_dot_est(i-1,k) + x_ddot_est(i-1,k) * delta_t(i-1) + H/delta_t(i-1) * (x(i) - tmp);
            x_ddot_est(i,k) = x_ddot_est(i-1,k) + 2 * K / delta_t(i-1)^2 * (x(i) - tmp);
        end
    end
end

  