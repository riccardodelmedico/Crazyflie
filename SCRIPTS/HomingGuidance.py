import math
import threading
import time
import numpy as np
import target_class as tar_c
import matplotlib.pyplot as plt
import FadingFilterHoming as FFH


def sim_homing_guidance(pursuer, target, dt, n):
    (homing_guidance, homing_guidance_dot, los_rate) = target.get_estimation(pursuer.p, pursuer.v, pursuer.old_acc)

    if homing_guidance[1] < 5e-2:
        print(f" interception at time: {target.time_line[-1] - target.time_line[0]},"
              f" R value at interception: {homing_guidance[1]}")
        tar_c.run = False

    # calculate PNG acceleration
    # print(f'R_dot: {homing_guidance_dot[1]}'
    #       f' sigma_dot: {homing_guidance_dot[0] + pursuer.old_acc/pursuer.chase_velocity}')

    acc = n * (-homing_guidance_dot[1]) * (los_rate + pursuer.old_acc / pursuer.chase_velocity)#(homing_guidance_dot[0] + pursuer.old_acc / pursuer.chase_velocity)
    print(f'[commanded acceleration]:{acc}')
    pursuer.old_acc = acc
    if acc > 500:
        acc = 0
        tar_c.run = False
    # update pursuer position and velocity with Forward Euler
    if np.linalg.norm(pursuer.v, 2) != 0 and tar_c.run is True:
        pursuer.p += pursuer.v * dt + \
                     target.target.omega_vers_hat.dot(pursuer.v / np.linalg.norm(pursuer.v, 2)) * acc * dt * dt / 2
        pursuer.v += target.target.omega_vers_hat.dot(pursuer.v / np.linalg.norm(pursuer.v, 2)) * acc * dt

    pursuer.list_pos = np.concatenate((pursuer.list_pos, pursuer.p.reshape((1, 3))), axis=0)


class Guidance:
    def __init__(self, target, initial_pose=np.array([0.0, 0.0, 0.5, math.pi / 2]), chase_vel=0.2, n=3, dt=0.05):
        # pursuer position  and velocity
        self.p = np.array(initial_pose[0:3])
        # print(initial_pose[3])
        self.v = np.array([math.cos(initial_pose[3]), math.sin(initial_pose[3]), 0.0]) * chase_vel
        # print(f 'pursuer initial Pos:{self.p},initial Vel:{self.v}')

        # list of element for plot
        self.list_pos = self.p.reshape((1, 3))
        self.update_thread = threading.Thread(target=tar_c.repeat_fun,
                                              args=(dt, sim_homing_guidance, self, target, dt, n))
        self.old_acc = 0
        self.target = target
        self.target.initialize(self.p, self.v)
        self.N = n
        self.chase_velocity = chase_vel

    def start(self):
        tar_c.run = True
        self.update_thread.start()
        self.target.start()

    def stop(self):
        tar_c.run = False
        self.update_thread.join()
        self.target.stop()

    def plot_chase(self):
        plt.figure(1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(self.list_pos[0, 0], self.list_pos[0, 1], 'ro')
        plt.plot(self.list_pos[:, 0], self.list_pos[:, 1], 'r-')
        plt.plot(self.list_pos[-1, 0], self.list_pos[-1, 1], 'r->')
        plt.plot(self.target.real_target_pos[0, 0], self.target.real_target_pos[0, 1], 'bo')
        plt.plot(self.target.real_target_pos[:, 0], self.target.real_target_pos[:, 1], 'g-')
        plt.plot(self.target.real_target_pos[-1, 0], self.target.real_target_pos[-1, 1], 'g->')
        plt.show()


in_p = np.array([10.0, 10.0, 0.5])
in_v = np.array([-1.0, -1.0, 0.0])
in_a = np.zeros(3)
delta = 4e-2
chase_velocity = 3

a = tar_c.Target(initial_position=in_p, initial_velocity=in_v, initial_acceleration_module=0.5, dt=0.002)
ff = FFH.FadingFilterHoming(a, chase_velocity, std=3e-4, order=2, beta=0.2, dt=delta)

gamma_p = math.pi/2
ini_pur_pos = np.array([0.0, 0.0, 0.5, gamma_p])

pur = Guidance(ff, initial_pose=ini_pur_pos, chase_vel=chase_velocity, dt=delta, n=4)

pur.start()
time.sleep(8)
pur.stop()
ff.plot_estimate_homing()
pur.plot_chase()
