function [sm_x_dot, sm_x_ddot, x_dot, x_ddot] = smoothing(x, param, time)
%param == dimension of moving average

difference = diff(x);
delta_t = (diff(time));
x_dot = difference./delta_t;
sm_x_dot = smooth(x_dot, param, 'loess');
difference_2 = diff(sm_x_dot);
x_ddot = difference_2./delta_t(2:end);
sm_x_ddot = smooth(x_ddot, param, 'loess');

end



    