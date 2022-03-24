function [sm_x_dot, sm_x_ddot, x_dot, x_ddot] = smoothing(x, param, time)
%smoothing function uses MATLAB smooth function to obtain quantities which
% can be used as references
%   The function obtains the first and second derivative of x; it returns
%   smoothed quantities (sm_x_dot, sm_x_ddot) but also the numerical
%   derivatives (x_dot, x_ddot). Note that x_ddot is computed using 
%   smoothed first derivative to avoid excessive noise 
%   param: dimension of moving average
%   time: time vector used for differentiating
difference = diff(x);
delta_t = (diff(time));
x_dot = difference./delta_t;
sm_x_dot = smooth(x_dot, param, 'loess');
difference_2 = diff(sm_x_dot);
x_ddot = difference_2./delta_t(2:end);
sm_x_ddot = smooth(x_ddot, param, 'loess');

end



    