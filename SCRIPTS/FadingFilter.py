import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import Target as t_c


class FadingFilter:
    def __init__(self, nstd=3e-3, dimensions=2, order=2, beta=0.5, dt=0.05):
        self.order = order
        self.dt = dt
        self.dim = dimensions
        self.std = nstd
        if order < 1 or order > 3:
            print('It is not possible create a filter with this order')
            return 0
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
    def initialization(self, initial_estimation):
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
                        self.x_ddot_est = initial_estimation[3, :]

    def compute(self, measure):
        if measure.shape[0] != self.x_est.shape[0]:
            print('Incorrect Dimension of Measure Tensor')
            return False

        if self.order == 1:
            x_est_next = self.x_est + self.g * (measure - self.x_est)

        if self.order == 2:
            tmp = self.x_est + self.dt * self.x_dot_est
            x_est_next = tmp + self.g * (measure - tmp)
            x_dot_est_next = self.x_dot_est + (self.h / self.dt) * (
                        measure - tmp)

        if self.order == 3:
            tmp = self.x_est + self.dt * self.x_dot_est + 0.5 * self.x_ddot_est * math.pow(
                self.dt, 2)
            x_est_next = tmp + self.g * (measure - tmp)
            x_dot_est_next = self.x_dot_est + self.dt * self.x_ddot_est + (
                        self.h / self.dt) * (measure - tmp)
            x_ddot_est_next = self.x_ddot_est + (
                        (2 * self.k) / math.pow(self.dt, 2)) * (measure - tmp)

            self.x_ddot_est = x_ddot_est_next

        self.x_est = x_est_next
        self.x_dot_est = x_dot_est_next

        return x_est_next, x_dot_est_next, x_ddot_est_next

