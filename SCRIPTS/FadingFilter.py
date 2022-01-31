import numpy as np
import math


class FadingFilter:
    def __init__(self, dimensions=1, order=2, beta=0.5):
        self.order = order
        self.dim = dimensions
        if order < 1 or order > 3:
            raise Exception("Order of Fading Filter must be between 1 and 3.")
        self.old_t = 0
        self.x_est = np.zeros(dimensions)
        self.x_dot_est = np.zeros(dimensions)
        self.time_line = 0

        if order == 1:
            self.g = 1 - beta

        if order > 1:
            self.g = 1 - math.pow(beta, 2)
            self.h = math.pow((1 - beta), 2)

        if order > 2:
            self.x_ddot_est = np.zeros(dimensions)
            self.g = 1 - math.pow((1 - beta), 3)
            self.h = 1.5 * math.pow((1 - beta), 2) * (1 + beta)
            self.k = 0.5 * math.pow((1 - beta), 3)

    # Filter initialization with the available measure
    def init(self, initial_estimation, initial_time):
        if initial_estimation.shape[0] != self.order or \
                initial_estimation.shape[1] != self.x_est.shape[0]:
            print('Incorrect Dimension of Initial Estimate Variable Tensor')
            return False
        else:
            if self.order >= 1:
                self.x_est = initial_estimation[0, :]
                if self.order >= 2:
                    self.x_dot_est = initial_estimation[1, :]
                    if self.order == 3:
                        self.x_ddot_est = initial_estimation[2, :]
            self.old_t = initial_time

            # print(f'x_est_ini: {self.x_est}, dot_x_est_ini {self.x_dot_est}')


    # Update step of the Fading Filter depending on its order
    # timestamp: time at which the measure are obtained; it's used for computing deltaT of the filter
    def update(self, measure, timestamp):
        if measure.shape[0] != self.x_est.shape[0]:
            print('Incorrect Dimension of Measure Tensor')
            return False
        dt = timestamp - self.old_t
        # print(f'timestamp :{timestamp}, dt : {dt}')
        self.old_t = timestamp
        if self.order == 1:
            x_est_next = self.x_est + self.g * (measure - self.x_est)
            self.x_est = x_est_next
            return self.x_est

        if self.order == 2:
            tmp = self.x_est + dt * self.x_dot_est
            self.x_est = tmp + self.g * (measure - tmp)
            x_dot_est_next = self.x_dot_est + (self.h / dt) * (
                        measure - tmp)
            self.x_dot_est = x_dot_est_next
            return self.x_est, self.x_dot_est

        if self.order == 3:
            tmp = self.x_est + dt * self.x_dot_est + 0.5 * self.x_ddot_est * math.pow(
                dt, 2)
            self.x_est = tmp + self.g * (measure - tmp)
            x_dot_est_next = self.x_dot_est + dt * self.x_ddot_est + (
                        self.h / dt) * (measure - tmp)
            x_ddot_est_next = self.x_ddot_est + (
                        (2 * self.k) / math.pow(dt, 2)) * (measure - tmp)
            self.x_dot_est = x_dot_est_next
            self.x_ddot_est = x_ddot_est_next
            return self.x_est, self.x_dot_est, self.x_ddot_est
