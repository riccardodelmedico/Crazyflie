import datetime
import logging
import threading
import time
import numpy as np
from FadingFilter import FadingFilter
from Seeker import *
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream


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


def guidance_png_command(guidance, n, dt, r_interception):
    # get the state of target
    (tar_pos, _, _, tar_time) = guidance.target.get_state()

    # evaluate the estimation of target quantities
    (est_t_pos, est_t_vel, est_t_acc) = guidance.target_ff.update(tar_pos, tar_time)

    # get the state of drone in world frame
    guidance.drone.get_state()
    rot_yaw = rot_z(guidance.drone.yaw)
    guidance.drone.velocity = rot_yaw.dot(guidance.drone.velocity)

    # closed form guidance quantities evaluate
    r = compute_r(guidance.drone.position, est_t_pos)
    sigma = compute_sigma(guidance.drone.position, est_t_pos)
    r_dot = compute_r_dot(guidance.drone.position, guidance.drone.velocity, est_t_pos, est_t_vel, r)
    sigma_dot = compute_sigma_dot(guidance.drone.position, guidance.drone.velocity, est_t_pos, est_t_vel, r)
    
    if r < r_interception and guidance.interception is None:
        guidance.drone.datalog.stop()
        crazy.run = False
        guidance.interception = r
        print(f"R value at interception: {r}")
        guidance.drone.landing()

    # get guidance quantities estimation
    (est_r, est_dot_r) = guidance.r_ff.update(np.array([r]), tar_time)
    (est_sigma, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]), tar_time)

    # conversion from python time to MATLAB time and write to logfile
    matlab_time = datetime.datetime.fromtimestamp(tar_time)
    matlab_time = f'{str(crazy.datetime2matlabdatenum(matlab_time))}'
    crazy.guidance_matlab.write(est_t_pos[0], est_t_pos[1], r, sigma, r_dot, sigma_dot,
                                est_r[0], est_sigma[0], est_dot_r[0], est_dot_sigma[0], matlab_time,
                                est_t_acc[0], est_t_acc[1])

    n_tv = 1 / math.cos(math.radians(guidance.drone.yaw) - sigma)

    # prova APNG mr. cioni
    # r_v = (est_t_pos[0:2] - guidance.drone.position[0:2])/r
    # r_v = np.append(r_v, 0)
    # r_ort = guidance.target.omega_vers_hat.dot(r_v)
    # apng_acc = np.transpose(r_ort).dot(est_t_acc)
    w = 0.5
    # calculate PNG acceleration with closed form quantities
    acc = n_tv * (2 * r_dot * est_dot_sigma + n * est_dot_sigma +
                  w * crazy.sign(est_dot_sigma))
    omega = - math.degrees(acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))
    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120

    guidance.send_command(guidance.v, 0.0, omega, dt=dt)


def guidance_png_homing(guidance, n, dt, r_interception):
    # get the state of target
    (tar_pos, _, _, tar_time) = guidance.target.get_state()

    # evaluate the estimation of target quantities
    (est_t_pos, est_t_vel, est_t_acc) = guidance.target_ff.update(tar_pos, tar_time)

    # get the state of drone in world frame
    guidance.drone.get_state()
    rot_yaw = rot_z(guidance.drone.yaw)
    guidance.drone.velocity = rot_yaw.dot(guidance.drone.velocity)

    # closed form guidance quantities evaluate
    r = compute_r(guidance.drone.position, est_t_pos)
    sigma = compute_sigma_homing(guidance.drone.position,
                                 guidance.drone.yaw, est_t_pos)

    r_dot = compute_r_dot(guidance.drone.position, guidance.drone.velocity,
                          est_t_pos, est_t_vel, r)
    sigma_dot = compute_sigma_dot(guidance.drone.position,
                                  guidance.drone.velocity, est_t_pos,
                                  est_t_vel, r)

    if r < r_interception and guidance.interception is None:
        guidance.drone.datalog.stop()
        crazy.run = False
        guidance.interception = r
        print(f"R value at interception: {r}")
        guidance.drone.landing()

    # get guidance quantities estimation
    (est_r, est_dot_r) = guidance.r_ff.update(np.array([r]), tar_time)
    (est_sigma, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]),
                                                          tar_time)
    yaw_radians = math.radians(guidance.drone.yaw)
    (est_yaw,est_yawrate) = guidance.yaw_ff.update(np.array([yaw_radians]),tar_time)

    # sum of sigma_body and yaw-rate to obtain sigma_dot "world"
    # est_dot_sigma += est_yawrate

    # conversion from python time to MATLAB time
    matlab_time = datetime.datetime.fromtimestamp(tar_time)
    matlab_time = f'{str(crazy.datetime2matlabdatenum(matlab_time))}'

    # write in Log [closed_form quantities, estimated quantities and the time of data acquisition]
    crazy.guidance_matlab.write(est_t_pos[0], est_t_pos[1], r, sigma, r_dot,
                                sigma_dot,
                                est_r[0], est_sigma[0], est_dot_r[0],
                                est_dot_sigma[0], matlab_time, yaw_radians
                                , guidance.drone.yawrate, est_yaw[0],est_yawrate[0] )

    n_tv = n / math.cos(sigma - math.radians(guidance.drone.yaw))

    # prova APNG mr. cioni
    r_v = (est_t_pos[0:2] - guidance.drone.position[0:2])/r
    r_ort = guidance.target.omega_vers_hat.dot(r_v)
    apng_acc = np.transpose(r_ort).dot(est_t_acc)

    # calculate PNG acceleration with closed form quantities
    acc = - n * (est_dot_r * est_dot_sigma) + apng_acc#+ np.linalg.norm(est_t_acc[0:2],2)/2
    omega = - math.degrees(acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))
    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120

    guidance.send_command(guidance.v, 0.0, omega, dt=dt)


class DroneGuidance:
    def __init__(self, guidance_ff_beta, target_ff_beta, target, drone_manager,
                 guidance_velocity=0.5, dt=0.05, N=3):
        self.interception = None
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(dt, guidance_png_command,
                                                    self, N, dt, 0.05))
        self.v = guidance_velocity
        self.drone = drone_manager
        self.target_ff = FadingFilter(dimensions=3, order=3, beta=target_ff_beta)
        self.target = target
        self.r_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)
        self.sigma_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)
        self.yaw_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)

    def start(self, vx, vy):
        self.drone.start(vx, vy)
        # take off deve eseguire finche non rispetta le condizioni di lancio
        # della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')
        if self.target.wand is True:
            in_p = get_wand_position()
            self.target.init_wand(in_p)
        else:
            in_p = self.target.pos
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)
        # init wand position

        # init target fading filter with initial state
        in_time = time.time()
        self.target_ff.init(np.array([in_p, in_v, in_a]), in_time)

        # initialization of Guidance Fading Filter:
        self.drone.get_state()
        rot_yaw = rot_z(self.drone.yaw)
        drone_position = self.drone.position
        drone_velocity = rot_yaw.dot(self.drone.velocity)
        in_sigma = compute_sigma(self.drone.position, in_p)
        # in_sigma = compute_sigma_homing(self.drone.position, self.drone.yaw, in_p)
        in_r = compute_r(self.drone.position, in_p)
        in_sigma_dot = compute_sigma_dot(drone_position, drone_velocity, in_p, in_v, in_r)
        in_r_dot = compute_r_dot(drone_position, drone_velocity, in_p, in_v, in_r)
        self.r_ff.init(np.array([[in_sigma], [in_sigma_dot]]), in_time)
        self.sigma_ff.init(np.array([[in_r], [in_r_dot]]), in_time)
        yaw_radians = math.radians(self.drone.yaw)
        self.yaw_ff.init(np.array([[yaw_radians], [self.drone.yawrate]]), in_time)
        #time.sleep(0.01)
        crazy.run = True
        self.target.start()
        self.update_thread.start()

    def stop(self):
        # tar_c.run = False
        self.update_thread.join()
        self.target.stop()

    def send_command(self, vx, vy, omega, dt=0.05):
        # dt: send_hover_setpoint frequency
        self.drone.send_command(vx, vy, omega, dt)




