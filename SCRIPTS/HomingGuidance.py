import math
import threading
import time
import numpy as np
import target_class as tar_c
import matplotlib.pyplot as plt
import Fading_Filter_Homing as FFH
from own_module import crazyfun
from own_module import script_variables as sc_v
from own_module import script_setup as sc_s
import logging
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import DroneManager as DM


def sim_homing_guidance(pursuer, target, dt, n):
    pursuer.drone.get_state()
    (homing_guidance, homing_guidance_dot, los_rate) = target.get_estimation(
        pursuer.drone.position, math.radians(pursuer.drone.yaw),
        pursuer.drone.yawrate)

    if homing_guidance[1] < 5e-2:
        print(
            f" interception at time: {target.time_line[-1] - target.time_line[0]},"
            f" R value at interception: {homing_guidance[1]}")
        crazyfun.run = False
        pursuer.drone.landing()

    # calculate PNG acceleration
    # print(f'R_dot: {homing_guidance_dot[1]}'
    #       f' sigma_dot: {homing_guidance_dot[0] + pursuer.old_acc/pursuer.chase_velocity}')
    # N_tv = N / math.cos(sigma - math.radians(pursuer.drone.yaw))
    print(f' Vc: {-homing_guidance_dot[1]} LOSrate: {los_rate + pursuer.drone.yawrate}')
    print(f'il valore di ')
    acc = n * (-homing_guidance_dot[1]) * (
                los_rate + pursuer.drone.yawrate)  # (homing_guidance_dot[0] + pursuer.old_acc / pursuer.chase_velocity)
    omega = -math.degrees(acc / np.linalg.norm(pursuer.drone.velocity[0:2], 2))
    print(f'[commanded acceleration]:{acc}')
    print(
        f'[command yawrate]: {omega}, vel_mod: {np.linalg.norm(pursuer.drone.velocity[0:2], 2)}')
    pursuer.old_acc = acc
    # if omega > 120:
    #     omega = 120
    #     # acc = 0
    # elif omega < -120:
    #     omega = 120

    # update pursuer position and velocity with Forward Euler
    t_pos = pursuer.target.real_target_pos[-1]
    crazyfun.guidance_matlab.write(t_pos[0], t_pos[1], homing_guidance[1],
                                   -homing_guidance_dot[1],
                                   los_rate + pursuer.drone.yawrate)
    pursuer.send_command(pursuer.v, 0.0, omega, dt=dt)
    pursuer.list_pos = np.concatenate(
        (pursuer.list_pos, pursuer.drone.position.reshape((1, 2))), axis=0)


class Guidance:
    def __init__(self, target, droneM, chase_vel=0.2, n=3, dt=0.05):

        # print(f 'pursuer initial Pos:{self.p},initial Vel:{self.v}')
        self.drone = droneM
        # list of element for plot
        self.list_pos = np.array([-100.0, -100.0]).reshape((1, 2))
        self.update_thread = threading.Thread(target=crazyfun.repeat_fun,
                                              args=(
                                              dt, sim_homing_guidance, self,
                                              target, dt, n))
        self.old_acc = 0
        self.v = chase_vel
        self.target = target
        # self.target.initialize(self.p, self.v)
        self.N = n
        self.chase_velocity = chase_vel

    # def start(self):
    #     crazyfun.run = True
    #     self.update_thread.start()
    #     self.target.start()
    #
    # def stop(self):
    #     crazyfun.run = False
    #     self.update_thread.join()
    #     self.target.stop()
    def start(self, vx, vy):
        self.drone.start(vx, vy)
        # take off deve eseguire finche non rispetta le condizioni di lancio
        # della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')

        try:
            sc_s.vicon.GetFrame()
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", str(exc))

            # Get current drone position and orientation in Vicon
        sc_v.wand_pos = sc_s.vicon. \
            GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

        # Converts in meters
        in_p = np.array([float(sc_v.wand_pos[0] / 1000),
                         float(sc_v.wand_pos[1] / 1000),
                         float(sc_v.wand_pos[2] / 1000)])
        print("terminata l'inizializzaizone reale")
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)
        self.drone.get_state()
        cos_yaw = math.cos(math.radians(self.drone.yaw))
        sin_yaw = math.sin(math.radians(self.drone.yaw))
        rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        self.drone.velocity = rot_yaw.dot(self.drone.velocity)
        self.target.target.initial_wand(in_p)
        print(
            f'Posizioni e velocita iniziali in terna World {self.drone.position},{self.drone.velocity}')
        self.target.initialize(self.drone.position, self.drone.velocity)
        crazyfun.run = True
        self.target.start()
        self.update_thread.start()

    def stop(self):
        # tar_c.run = False
        self.update_thread.join()
        self.target.stop()

    def send_command(self, vx, vy, omega, dt=0.05):
        self.drone.send_command(vx, vy, omega, dt)

    def plot_chase(self):
        self.list_pos = self.list_pos[1:-1, :]
        print(f'lunghezza di list_pos:{len(self.list_pos)}')
        plt.figure(1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(self.list_pos[0, 0], self.list_pos[0, 1], 'ro')
        plt.plot(self.list_pos[:, 0], self.list_pos[:, 1], 'r-')
        plt.plot(self.list_pos[-1, 0], self.list_pos[-1, 1], 'r->')
        plt.plot(self.target.real_target_pos[0, 0],
                 self.target.real_target_pos[0, 1], 'bo')
        plt.plot(self.target.real_target_pos[:, 0],
                 self.target.real_target_pos[:, 1], 'g-')
        plt.plot(self.target.real_target_pos[-1, 0],
                 self.target.real_target_pos[-1, 1], 'g->')
        plt.show()


# in_p = np.array([10.0, 10.0, 0.5])
# in_v = np.array([-1.0, -1.0, 0.0])
# in_a = np.zeros(3)
# delta = 4e-2
# chase_velocity = 2.5
#
# a = tar_c.target(initial_position=in_p, initial_velocity=in_v, initial_acceleration_module=0.5, dt=0.002)
# ff = FFH.FadingFilterHoming(a, chase_velocity, std=3e-4, order=2, beta=0.2, dt=delta)
#
# gamma_p = math.pi/2
# ini_pur_pos = np.array([0.0, 0.0, 0.5, gamma_p])
#
# pur = Guidance(ff, initial_pose=ini_pur_pos, chase_vel=chase_velocity, dt=delta, n=5)
#
# pur.start()
# time.sleep(8)
# pur.stop()
# ff.plot_estimate_homing()
# pur.plot_chase()

print('inizializzo il vettore delle posizioni iniziali a zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

        # Get current drone position and orientation in Vicon
    sc_v.wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters
    in_p = np.array([float(sc_v.wand_pos[0] / 1000),
                     float(sc_v.wand_pos[1] / 1000),
                     float(sc_v.wand_pos[2] / 1000)])
print("terminata l'inizializzaizone reale")
in_v = np.array([0.0, 0.0, 0.0])
in_a = np.zeros(3)
delta = 2e-2

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    # comp = Model_Compensation(np.array([0.0299875609518963, 0.0300702032877777, 3.00672045316825e-06, 0]),
    #                           np.array([1, -0.516503065398502, -0.970604433271622, 0.545898677743298]))
    vc = 0.6
    target = tar_c.target(initial_position=in_p,
                          dt=0.1, use_wand_target=True)
    ff = FFH.FadingFilterHoming(target, chase_vel=vc, std=0, beta=0.1, dt=delta)

    drone = DM.DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                            box=np.array([1.0, -1.8, 2.0, -0.5]))

    guidance = Guidance(ff, drone, chase_vel=vc, n=4, dt=delta)
    guidance.start(0.0, 0.6)
    guidance.stop()
    # guidance.plot_chase()
    ff.plot_estimate_homing()
    guidance.plot_chase()

    print('Main Finished')
# test venuti egregiamente :
