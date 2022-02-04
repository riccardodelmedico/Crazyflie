import datetime
import logging
import math
import threading
import time
import numpy as np
from FadingFilter import FadingFilter
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream
from DroneGuidance import DroneGuidance,compute_r,compute_r_dot,compute_sigma,compute_sigma_dot,rot_z,get_wand_position


def guidance_apng(guidance, n, dt, r_interception):
    # get the state of target

    (tar_pos, _, _, tar_time) = guidance.target.get_state()

    # evaluate the estimation of target quantities
    (est_t_pos, est_t_vel, _) = guidance.target_ff.update(tar_pos, tar_time)
    # print(f'target_time: {tar_time}')
    # print(f'target estimated Velocity : {est_t_vel}')
    # get the state of drone in world frame

    guidance.drone.get_state()
    rot_yaw = rot_z(guidance.drone.yaw)
    guidance.drone.velocity = rot_yaw.dot(guidance.drone.velocity)
    # print(f'drone velocity in guidance: {guidance.drone.velocity}')
    # print(f'drone position in guidance:{guidance.drone.position}')
    # closed form guindance quantities evaluate
    r = compute_r(guidance.drone.position, est_t_pos)
    sigma = compute_sigma(guidance.drone.position, est_t_pos)
    r_dot = compute_r_dot(guidance.drone.position, guidance.drone.velocity,
                          est_t_pos, est_t_vel, r)
    sigma_dot = compute_sigma_dot(guidance.drone.position,
                                  guidance.drone.velocity, est_t_pos, est_t_vel,
                                  r)

    if r < r_interception and guidance.interception is None:
        guidance.drone.datalog.stop()
        crazy.run = False
        guidance.interception = r
        print(f"R value at interception: {r}")
        guidance.drone.landing()
    # get guidance quatities estimation
    (est_r, est_dot_r) = guidance.r_ff.update(np.array([r]), tar_time)
    (est_sigma, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]),
                                                          tar_time)
    (est_yr,est_yr_dot,est_yr_ddot) = guidance.yr_ff.update(np.array([r*math.cos(sigma)]),tar_time)
    # conversion from python time to MATLAB time
    matlab_time = datetime.datetime.fromtimestamp(tar_time)
    matlab_time = f'{str(crazy.datetime2matlabdatenum(matlab_time))}'

    # write in Log [closed_form quantities, estimated quantities and the time of data acquisition]
    crazy.guidance_matlab.write(est_t_pos[0], est_t_pos[1], r, sigma, r_dot,
                                sigma_dot,
                                est_r[0], est_sigma[0], est_dot_r[0],
                                est_dot_sigma[0], matlab_time)

    n_tv = n / math.cos(sigma - math.radians(guidance.drone.yaw))
    # calculate PNG acceleration with closed form quantities
    acc = - n * (est_dot_r * est_dot_sigma   + est_yr_ddot/2)
    omega = - math.degrees(
        acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))
    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120

    guidance.send_command(guidance.v, 0.0, omega, dt=dt)

class GuidanceAPNG(DroneGuidance):
    def __init__(self,guidance_ff_beta, target_ff_beta, yr_beta, target, drone_manager, guidance_velocity=0.5,
                 dt=0.05, N=3):
        super().__init__(guidance_ff_beta, target_ff_beta, target, drone_manager, guidance_velocity,
                 dt, N)
        self.yr_ff = FadingFilter(order=3, dimensions=1,
                                     beta=yr_beta)
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(
                                              dt, guidance_apng, self, N, dt,
                                              0.05))
    def start(self, vx, vy):
        self.drone.start(vx, vy)
        # take off deve eseguire finche non rispetta le condizioni di lancio
        # della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')

        in_p = get_wand_position()
        #
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)
        # init wand position
        self.target.init_wand(in_p)
        # init target fading filter with initial state
        in_time = time.time()
        self.target_ff.init(np.array([in_p, in_v, in_a]), in_time)

        # inizialization of Homing Fading Filter:
        self.drone.get_state()
        rot_yaw = rot_z(self.drone.yaw)
        drone_position = self.drone.position
        drone_velocity = rot_yaw.dot(self.drone.velocity)
        in_sigma = compute_sigma(self.drone.position, in_p)
        in_r = compute_r(self.drone.position, in_p)
        in_sigma_dot = compute_sigma_dot(drone_position, drone_velocity, in_p, in_v, in_r)
        in_r_dot = compute_r_dot(drone_position, drone_velocity, in_p, in_v, in_r)
        # print(f' i valori della sigma dot e della R_dot iniziali sono {in_sigma_dot,in_r_dot}')
        self.r_ff.init(np.array([[in_sigma], [in_sigma_dot]]), in_time)
        self.sigma_ff.init(np.array([[in_r], [in_r_dot]]), in_time)
        yr = in_r * math.cos(in_sigma)
        yr_dot = in_r_dot*in_sigma + in_r*in_sigma_dot
        self.yr_ff.init(np.array([[yr],[yr_dot],[0.0]]),in_time)
        crazy.run = True
        self.target.start()
        self.update_thread.start()