import datetime
import logging
import math
import threading
import time
import numpy as np
from FadingFilter import FadingFilter
from GuidanceUtility import *
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream
guidance_data = np.zeros((3,1))

def get_wand_position():
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
    return in_p


def rot_z(yaw, radians=False):
    if not radians:
        rad_yaw = math.radians(yaw)
    else:
        rad_yaw = yaw
    cos_yaw = math.cos(rad_yaw)
    sin_yaw = math.sin(rad_yaw)
    rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    return rot_yaw


# def guidance_png_command(guidance, n, dt, r_interception):
#     # get the state of target
#     (tar_pos, _, _, tar_time) = guidance.target.get_state()
#
#     # evaluate the estimation of target quantities
#     (est_t_pos, est_t_vel, est_t_acc) = guidance.target_ff.update(tar_pos, tar_time)
#
#     # get the state of drone in world frame
#     guidance.drone.get_state()
#     rot_yaw = rot_z(guidance.drone.yaw)
#     guidance.drone.velocity = rot_yaw.dot(guidance.drone.velocity)
#
#     # closed form guidance quantities evaluate
#     r = compute_r(guidance.drone.position, est_t_pos)
#     sigma = compute_sigma(guidance.drone.position, est_t_pos)
#     r_dot = compute_r_dot(guidance.drone.position, guidance.drone.velocity, est_t_pos, est_t_vel, r)
#     sigma_dot = compute_sigma_dot(guidance.drone.position, guidance.drone.velocity, est_t_pos, est_t_vel, r)
#     yr = r * sigma
#     (est_yr, est_dot_yr, est_ddot_yr) = guidance.yr_ff.update(np.array([yr]), tar_time)
#
#     if r < r_interception and guidance.interception is None:
#         guidance.drone.datalog.stop()
#         crazy.run = False
#         guidance.interception = r
#         print(f"R value at interception: {r}")
#         guidance.drone.landing()
#
#     # get guidance quantities estimation
#     (est_r, est_dot_r) = guidance.r_ff.update(np.array([r]), tar_time)
#     (est_sigma, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]), tar_time)
#     # print(f' valore della sigma dot calcolata vs. stimata {sigma_dot,est_dot_sigma}')
#
#     n_tv = 1 / math.cos(math.radians(guidance.drone.yaw) - sigma)
#
#     # prova APNG mr. cioni
#     r_v = (est_t_pos[0:2] - guidance.drone.position[0:2])/r
#     r_v = np.append(r_v, 0)
#     r_ort = guidance.target.omega_vers_hat.dot(r_v)
#     apng_acc = np.transpose(r_ort).dot(est_t_acc)
#     acc = - n * est_dot_sigma * est_dot_r + apng_acc/2
#     # # Sliding mode guidance
#     # W = 0.5
#     # acc = n_tv * (2 * r_dot * est_dot_sigma + n * est_dot_sigma +
#     #               w * crazy.sign(est_dot_sigma))
#
#     # conversion from python time to MATLAB time and write to logfile
#     matlab_time = datetime.datetime.fromtimestamp(tar_time)
#     matlab_time = f'{str(crazy.datetime2matlabdatenum(matlab_time))}'
#     crazy.guidance_matlab.write(est_t_pos[0], est_t_pos[1], r, sigma, r_dot,
#                                 sigma_dot,
#                                 est_r[0], est_sigma[0], est_dot_r[0],
#                                 est_dot_sigma[0], matlab_time,
#                                 est_t_acc[0], est_t_acc[1], est_ddot_yr[0], apng_acc)
#
#     omega = - math.degrees(acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))
#     # omega saturation
#     if omega > 120:
#         omega = 120
#     elif omega < -120:
#         omega = -120
#
#     guidance.send_command(guidance.v, 0.0, omega, dt=dt)


def guidance_png_command(guidance, n, dt, r_interception):
    global guidance_data
    measures = guidance.seeker.get_measures()
    r = measures[0]
    sigma = measures[1]
    yr = r * sigma
    t_acc_x = measures[2]
    t_acc_y = measures[3]
    new_time = measures[-1]
    # print(f' il valore del tempo nella guida: {measures[-1]}')

    if r < r_interception and guidance.interception is None:
        guidance.drone.datalog.stop()
        crazy.run = False
        guidance.interception = r
        print(f"R value at interception: {r}")
        guidance.drone.landing()

    (_, est_dot_r) = guidance.r_ff.update(np.array([r]), new_time)
    (_, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]), new_time)
    (_, est_dot_yr, est_ddot_yr) = guidance.yr_ff.update(np.array([yr]),
                                                              new_time)

    acc_apng = t_acc_x * math.cos(sigma + math.pi/2) + t_acc_y * math.sin(sigma + math.pi/2)
    # acc_apng = 0
    acc = n * -est_dot_sigma * est_dot_r +  acc_apng/2
    guidance.drone.get_state()
    omega = - math.degrees(
        acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))


    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120
    guidance_data[0] = est_dot_r
    guidance_data[1] = est_dot_sigma
    guidance_data[2] = new_time
    guidance.logger.append(guidance_data)
    guidance.send_command(guidance.v, 0.0, omega, dt=dt)




class DroneGuidance:
    def __init__(self, guidance_ff_beta, yr_ff_beta, seeker, drone_manager,
                 guidance_velocity=0.5, dt=0.05, N=3):

        self.interception = None
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(1, dt, guidance_png_command,
                                                    self, N, dt, 0.05))
        self.v = guidance_velocity
        self.drone = drone_manager
        self.seeker = seeker
        self.r_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta[0])
        self.sigma_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta[1])
        self.yr_ff = FadingFilter(order=3, dimensions=1, beta=yr_ff_beta)
        self.logger = crazy.AsyncMatlabPrint(3,flag=5)
        # self.yaw_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)

    def start(self, pos, vel):
        self.drone.start(pos, vel)
        # take off deve eseguire finche non rispetta le condizioni di lancio
        # della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')
        time.sleep(0.02)
        # initialize filter inside DroneGuidance:
        init_measures = self.seeker.get_measures()
        print(f'init measures:{init_measures}')
        self.r_ff.init(np.array([[init_measures[0]],[0.0]]),init_measures[-1])
        self.sigma_ff.init(np.array([[init_measures[1]],[0.0]]),init_measures[-1])
        yr = init_measures[0]*init_measures[1]
        self.yr_ff.init(np.array([[yr],[0.0],[0.0]]),init_measures[-1])
        crazy.run = True
        time.sleep(0.02)
        self.update_thread.start()

    def stop(self):
        # tar_c.run = False
        self.update_thread.join()

    def send_command(self, vx, vy, omega, dt=0.05):
        # dt: send_hover_setpoint frequency
        self.drone.send_command(vx, vy, omega, dt)




