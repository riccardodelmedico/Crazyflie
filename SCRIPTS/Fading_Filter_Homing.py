import numpy as np
import math
import random
import time
import matplotlib.pyplot as mpl
from own_module import crazyfun


# Utility function for filter initialization
def compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = (target_vel[1] - pursuer_vel[1]) * (target_pos[0] - pursuer_pos[0]) - \
          (target_vel[0] - pursuer_vel[0]) * (target_pos[1] - pursuer_pos[1])
    den = r * r
    return num / den


def compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = ((target_vel[0] - pursuer_vel[0]) * (target_pos[0] - pursuer_pos[0]) +
           (target_vel[1] - pursuer_vel[1]) * (target_pos[1] - pursuer_pos[1]))
    den = r
    return num / den


class FadingFilterHoming:
    def __init__(self, target, chase_vel, std=3e-3, order=2, beta=0.5, dt=0.05):
        self.order = order
        self.dt = dt
        self.target = target
        self.std = std
        self.chase_vel = chase_vel
        if order <= 1 or order > 2:
            print('it is not possible create a filter with this order')
            return
        # guidance quantities: homing_est[0] = sigma, homing_est[1] = R
        self.homing_est = np.zeros(2, dtype='float')
        self.homing_dot_est = np.zeros(2, dtype='float')
        # real quantities
        self.real_target_pos = np.array([[0, 0]])
        self.real_homing = np.array([[0, 0]])
        self.real_homing_dot = np.array([[0, 0]])
        self.los_rate = np.array([0])
        self.r_rate = np.array([0])
        # plot quantities
        self.list_homing = np.array([[0, 0]])
        self.list_homing_dot = np.array([[0, 0]])
        self.gamma_dot = np.array([0])
        self.time_line = np.array([0])
        self.G = 1 - math.pow(beta, 2)
        self.H = math.pow((1 - beta), 2)

    def initialize(self, pursuer_pos, pursuer_vel):
        # get target and pursuer state for initialization
        (target_pos, target_vel, target_acc, t) = self.target.get_target()

        # compute R and sigma
        r = np.linalg.norm(target_pos[0:2] - pursuer_pos[0:2], 2)
        r_v = target_pos[0:2] - pursuer_pos[0:2]  # vector form
        sigma = math.atan2(r_v[1], r_v[0]) - math.atan2(pursuer_vel[1], pursuer_vel[0])
        self.homing_est[0] = sigma
        self.homing_est[1] = r
        # print(f' init: sigma:{sigma}, R:{r}')

        # compute R_dot and sigma_dot: their initialization is done with real data
        self.homing_dot_est[0] = self.los_rate = compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r)
        self.homing_dot_est[1] = self.r_rate = compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r)
        # print(f' init: sigma_dot:{self.homing_dot_est[0]}, R_dot:{self.homing_dot_est[1]}')

        self.list_homing = self.real_homing = self.homing_est.reshape((1, 2))
        self.list_homing_dot = self.real_homing_dot = self.homing_dot_est.reshape((1, 2))
        self.real_target_pos = np.array([target_pos[0:2]])
        self.time_line[0] = t
        self.gamma_dot[0] = 0.0

    def add_noise(self, target_pos):
        return target_pos[0:2] + np.array([random.gauss(0, self.std), random.gauss(0, self.std)])

    def start(self):
        self.time_line = np.array([time.time()])
        crazyfun.run = True
        self.target.start()

    def stop(self):
        crazyfun.run = False
        self.target.stop()
        self.time_line -= self.time_line[0]

    def update(self, pursuer_pos, pursuer_vel,pursuer_old_yawrate):
        (target_pos, target_vel, _, t) = self.target.get_target()
        target_pos_measured = self.add_noise(target_pos)

        r_v_real = target_pos[0:2] - pursuer_pos[0:2]
        r_real = np.linalg.norm(r_v_real, 2)
        r_v_measured = target_pos_measured[0:2] - pursuer_pos[0:2]
        homing_measured = np.array([math.atan2(r_v_measured[1], r_v_measured[0]) -
                                    math.atan2(pursuer_vel[1], pursuer_vel[0]),
                                    np.linalg.norm(r_v_measured, 2)])

        # fading filter update
        old_sigma = self.homing_est[0]
        old_r = self.homing_est[1]
        tmp = self.homing_est + self.dt * self.homing_dot_est
        next_homing_est = tmp + self.G * (homing_measured - tmp)
        self.homing_est = next_homing_est

        # numerical derivative of sigma and r
        self.los_rate = np.append(self.los_rate, (self.homing_est[0] - old_sigma)/self.dt)
        self.r_rate = np.append(self.r_rate, (self.homing_est[1] - old_r)/self.dt)

        next_homing_dot_est = self.homing_dot_est + (self.H / self.dt) * (homing_measured - tmp)
        next_homing_dot_est[0] = 1/2*self.los_rate[-1] + 1/2*next_homing_dot_est[0]
        next_homing_dot_est[1] = 1/2*self.r_rate[-1] + 1/2*next_homing_dot_est[1]
        self.homing_dot_est = next_homing_dot_est

        # print(f' [measures]: sigma:{homing_measured[0]} R:{homing_measured[1]}')
        # print(f'[sigma, R]_(k+1): {self.homing_est[0]}, {self.homing_est[1]}')
        # print(f'[sigma_dot, R_dot]_(k+1): {self.homing_dot_est[0], self.homing_dot_est[1]}')

        # real quantities
        homing_real = np.array([math.atan2(r_v_real[1], r_v_real[0]) -
                                math.atan2(pursuer_vel[1], pursuer_vel[0]),
                                r_real])
        homing_dot_real = np.array([compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r_real),
                                    compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r_real)])

        # update data for graphical visualization
        self.list_homing = np.concatenate((self.list_homing, next_homing_est.reshape((1, 2))), axis=0)
        self.list_homing_dot = np.concatenate((self.list_homing_dot, next_homing_dot_est.reshape((1, 2))), axis=0)
        self.real_homing = np.concatenate((self.real_homing, homing_real.reshape(1, 2)), axis=0)
        self.real_homing_dot = np.concatenate((self.real_homing_dot, homing_dot_real.reshape(1, 2)), axis=0)
        self.real_target_pos = np.concatenate((self.real_target_pos, target_pos[0:2].reshape((1, 2))), axis=0)
        self.time_line = np.append(self.time_line, t)
        self.gamma_dot = np.append(self.gamma_dot,pursuer_old_yawrate)

    def get_estimation(self, pursuer_pos, pursuer_vel, pursuer_old_yawrate):
        self.update(pursuer_pos, pursuer_vel, pursuer_old_yawrate)
        return self.list_homing[-1], self.list_homing_dot[-1], self.los_rate[-1]

    def plot_estimate_homing(self):
        legend = np.array(['Sigma', 'R', 'Sigma_dot', 'R_dot'])
        sp, ax = mpl.subplots(4, 1)
        sp.suptitle('Estimated Homing Guidance Quantities')
        ax[0].plot(self.time_line, self.list_homing[:, 0], '--b', label=f'estimate {legend[0]}')
        ax[0].plot(self.time_line, self.real_homing[:, 0], '--r', label=f'real {legend[0]}')
        ax[0].legend()
        ax[1].plot(self.time_line, self.list_homing[:, 1], '--b', label=f'estimate {legend[1]}')
        ax[1].plot(self.time_line, self.real_homing[:, 1], '--r', label=f'real {legend[1]}')
        ax[1].legend()
        ax[2].plot(self.time_line, self.list_homing_dot[:, 0] + self.gamma_dot, '-b', label=f'estimate {legend[2]}')
        ax[2].plot(self.time_line, self.real_homing_dot[:, 0], '-r', label=f'real {legend[2]}')
        ax[2].plot(self.time_line, self.los_rate + self.gamma_dot, '-g', label=f'numerical derivative {legend[2]}')
        ax[2].legend()
        ax[3].plot(self.time_line, self.list_homing_dot[:, 1], '-b', label=f'estimate {legend[3]}')
        ax[3].plot(self.time_line, self.real_homing_dot[:, 1], '-r', label=f'real {legend[3]}')
        ax[3].plot(self.time_line, self.r_rate, '-g', label=f'numerical derivative {legend[3]}')
        ax[3].legend()
        mpl.show()
        print(f'Minimum R: {min(self.list_homing[:, 1])}')