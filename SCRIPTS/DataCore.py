import logging
import threading
import numpy as np
from own_module import crazyfun as crazy
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s
from FadingFilter import FadingFilter
from DroneManager import set_phase, get_phase
import GuidanceUtility as gu

# number of guidance quantities which will be computed
data_dimension = 21

# mutex to access DataCore guidance_variable since they are shared between
# DataCore and Seeker
mutex = threading.Semaphore(value=1)
guidance_variable = np.zeros((data_dimension, 1))

# dictionary used to access DataCore guidance_variable
access_data = {
    'd_est_vel_x': 6,
    'd_est_vel_y': 7,
    'r': 8,
    'sigma': 9,
    'r_dot_ff': 10,
    'sigma_dot_ff': 11,
    'r_dot_kf': 12,
    'sigma_dot_kf': 13,
    'time': 14,
    't_acc_x': 15,
    't_acc_y': 16,
    'r_kf': 19,
    'sigma_kf': 20
}


def get_data(data_list):
    """
    Read a subsection of DataCore guidance_variable

    :param data_list: subsection of guidance_variable defined by dictionary
                      access_data
    :type data_list: string np.array[...]
    :return: subsection of guidance_variable and time
    :rtype: float np.array[...]
    """

    dim = len(data_list) + 1
    ret = np.zeros(dim)
    mutex.acquire(blocking=True)
    for i in range(dim - 1):
        ret[i] = guidance_variable[access_data[data_list[i]]]
    ret[-1] = guidance_variable[access_data['time']]
    mutex.release()
    return ret


def get_drone_kf_data():
    """
    Read the internal state of drone (from KF)

    :return: KF state (pos_x, pos_y, yaw, vel_x, vel_y, yaw_rate)
             in world reference frame.
    :rtype: float np.array[6]
    """
    ret = np.zeros(6)
    crazy.callback_mutex.acquire(blocking=True)
    pos_x = sc_v.pos_estimate[0]
    pos_y = sc_v.pos_estimate[1]
    yaw = sc_v.pos_estimate[2]
    yaw_rate = sc_v.vel_estimate[2] / 1000
    rot_yaw = gu.rot_z(yaw)
    ret[3:5] = rot_yaw.dot(sc_v.vel_estimate[0:2])
    crazy.callback_mutex.release()
    ret[0] = pos_x
    ret[1] = pos_y
    ret[2] = yaw
    ret[5] = yaw_rate
    return ret


def update_data_core(data):
    """
    DataCore thread body: it manages the interaction with Vicon, sends the
    position corrections to the drone and computes guidance quantities.

    :param data: instance of DataCore
    :type data: DataCore
    :return: None.
    :rtype: None
    """
    global guidance_variable, mutex
    guidance_phase = get_phase()
    # get the actual frame of vicon object
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

    # select drone position from frame
    drone_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    drone_pos = np.array([float(drone_pos[0] / 1000),
                          float(drone_pos[1] / 1000),
                          float(drone_pos[2] / 1000)])

    # update data time with frame number diff and save the current time
    frame_number = sc_s.vicon.GetFrameNumber()
    dt = 0
    mutex.acquire(blocking=True)
    if data.old_frame_number == 0:
        new_time = 0
    else:
        dt = (frame_number - data.old_frame_number) / data.vicon_freq
        new_time = guidance_variable[access_data['time']] + dt
    guidance_variable[access_data['time']] = new_time
    mutex.release()
    data.old_frame_number = frame_number

    # manage the sending of correction
    data.counter += 1
    if data.counter == 10:
        data.counter = 0
        # correction at 10Hz
        data.scf.cf.extpos.send_extpos(drone_pos[0], drone_pos[1],
                                       drone_pos[2])

    # drone has just entered into virtual box: fading filter of target and drone
    # must be initialized
    if guidance_phase == 1:
        # get actual position and velocity of target
        (t_pos, t_vel) = data.target.update(0)
        # init target fading filter
        print('init target filter')
        data.target_ff.init(np.array([t_pos[0:2], t_vel[0:2], np.zeros(2)]),
                            new_time)
        # init drone fading filter
        print('init drone filter')
        data.drone_ff.init(
            np.array([drone_pos[0:2], data.drone_vel, np.zeros(2)]),
            new_time)
        guidance_variable[access_data['r']] = gu.compute_r(drone_pos, t_pos)
        guidance_variable[access_data['sigma']] = gu.compute_sigma(drone_pos,
                                                                   t_pos)
        # switch to actual guidance
        set_phase(2)

    # guidance has actually began: guidance quantities, target and drone data
    # must be computed and saved
    elif guidance_phase == 2:
        # get target position
        (t_pos, t_vel) = data.target.update(dt)
        # get drone internal data
        internal = get_drone_kf_data()

        # compute measured R and sigma using only Vicon Frame
        r = gu.compute_r(drone_pos, t_pos)
        sigma = gu.compute_sigma(drone_pos, t_pos)

        # compute R and sigma using KF
        r_kf = gu.compute_r(internal[0:2], t_pos)
        sigma_kf = gu.compute_sigma(internal[0:2], t_pos)

        # update filter
        (t_est_pos, t_est_vel, t_est_acc) = data.target_ff.update(t_pos[0:2],
                                                                  new_time)
        (d_est_pos, d_est_vel, d_est_acc) = data.drone_ff.update(drone_pos[0:2],
                                                                 new_time)

        # compute derivative of R and sigma with FF data
        r_dot_ff = gu.compute_r_dot(drone_pos, d_est_vel, t_pos, t_est_vel, r)
        sigma_dot_ff = gu.compute_sigma_dot(drone_pos, d_est_vel, t_pos,
                                            t_est_vel, r)

        # compute derivative of R and sigma with KF data
        r_dot_kf = gu.compute_r_dot(internal[0:2], internal[3:5], t_pos,
                                    t_est_vel, r)
        sigma_dot_kf = gu.compute_sigma_dot(internal[0:2], internal[3:5], t_pos,
                                            t_est_vel, r)

        # update and log guidance_variable
        mutex.acquire(blocking=True)
        guidance_variable[0] = t_pos[0]
        guidance_variable[1] = t_pos[1]
        guidance_variable[2] = drone_pos[0]
        guidance_variable[3] = drone_pos[1]
        guidance_variable[4] = t_est_vel[0]
        guidance_variable[5] = t_est_vel[1]
        guidance_variable[6] = d_est_vel[0]
        guidance_variable[7] = d_est_vel[1]
        guidance_variable[access_data['r']] = r
        guidance_variable[access_data['sigma']] = sigma
        guidance_variable[access_data['r_dot_ff']] = r_dot_ff
        guidance_variable[access_data['sigma_dot_ff']] = sigma_dot_ff
        guidance_variable[access_data['r_dot_kf']] = r_dot_kf
        guidance_variable[access_data['sigma_dot_kf']] = sigma_dot_kf
        guidance_variable[access_data['t_acc_x']] = t_est_acc[0]
        guidance_variable[access_data['t_acc_y']] = t_est_acc[1]
        # guidance_variable[access_data['time']] = new_time
        guidance_variable[17] = d_est_acc[0]
        guidance_variable[18] = d_est_acc[1]
        guidance_variable[access_data['r_kf']] = r_kf
        guidance_variable[access_data['sigma_kf']] = sigma_kf
        data.logger.append(guidance_variable)
        mutex.release()


class DataCore:
    """
    DataCore manages Vicon and Drone data, their processing and computing with
    the correct timing

    Constructor:
    :param vicon_freq: frequency of Vicon Tracker
    :type vicon_freq: float
    :param target: instance of Target class
    :type target: Target
    :param beta: beta of target and drone fading filter;
                 beta[0] => target, beta[1] => drone
    :type beta: float np.array[2]
    :param drone_vel: initial drone velocity (for fading filter init.)
    :type drone_vel: np.array[2]
    :param scf: Synchronization wrapper of the Crazyflie object
    :type scf: SyncCrazyflie
    """

    def __init__(self, vicon_freq, target, beta, drone_vel, scf):
        self.vicon_freq = vicon_freq
        self.scf = scf
        self.counter = 0
        self.dt = 0.01
        self.old_frame_number = 0
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(0,
                                                    self.dt, update_data_core,
                                                    self))
        self.drone_vel = drone_vel
        self.target = target
        self.target_ff = FadingFilter(dimensions=2, order=3, beta=beta[0])
        self.drone_ff = FadingFilter(dimensions=2, order=3, beta=beta[1])
        self.logger = crazy.AsyncMatlabPrint(flag=6, num_data=data_dimension)

    def start(self):
        self.update_thread.daemon = True
        self.update_thread.start()

    def stop(self):
        self.update_thread.join()
