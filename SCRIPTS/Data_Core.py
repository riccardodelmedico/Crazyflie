import logging
import threading
import time
import numpy as np
from own_module import crazyfun as crazy
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s
from FadingFilter import FadingFilter
from DroneManager import set_phase,get_phase
import GuidanceUtility as gu

mutex = threading.Semaphore(value=1)
guidance_variable = np.zeros((19,1))

access_data = {
    'r': 8,
    'sigma': 9,
    'r_dot_ff': 10,
    'sigma_dot_ff': 11,
    'r_dot_kf': 12,
    'sigma_dot_kf': 13,
    'time': 14,
    't_acc_x': 15,
    't_acc_y': 16
}


# (est_t_pos[0], est_t_pos[1],est_pur_pos[0], est_pur_pos[1], && stessa cosa con le velocità, (4)
# r, sigma,
# r_dot,sigma_dot, (calcollate con V dai FF)
# r_dot,sigma_dot, (calcollate con V dai FF e Kalman fillter)
# matlab_time,
# est_t_acc[0], est_t_acc[1],est_p_acc[0], est_p_acc[1])




def get_data(list_param):
    dim = len(list_param) + 1
    ret = np.zeros((dim))
    mutex.acquire(blocking=True)
    for i in range(dim - 1):
        # print(f'paramentro {list_param[i]} vale {guidance_variable[access_data[list_param[i]]]}')
        ret[i] = guidance_variable[access_data[list_param[i]]]
    ret[-1] = guidance_variable[access_data['time']]
    mutex.release()
    return ret


def get_drone_kf_data():
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
    global guidance_variable, mutex
    phase = get_phase()
    # get the actual frame of viocn object
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

    # select Drone position from Frame
    drone_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    drone_pos = np.array([float(drone_pos[0] / 1000),
                 float(drone_pos[1] / 1000),
                 float(drone_pos[2] / 1000)])

    # Get current drone position and orientation in Vicon
    frame_number = sc_s.vicon.GetFrameNumber()

    # update data time with frame number diff and save the current time only
    # when the drone is out of the virtual box
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
    if phase == 1:
        # get actual position and velocity of target
        (t_pos, t_vel) = data.target.update(0)
        # init target fading filter
        print('init target filter')
        data.target_ff.init(np.array([t_pos[0:2], t_vel[0:2], np.zeros(2)]), new_time)
        # init drone fading filter
        print('init drone filter')
        data.drone_ff.init(np.array([drone_pos[0:2], data.drone_vel, np.zeros(2)]),
                           new_time)
        guidance_variable[access_data['r']] = gu.compute_r(drone_pos, t_pos)
        guidance_variable[access_data['sigma']] =  gu.compute_sigma(drone_pos, t_pos)
        set_phase(2)
    elif phase == 2:
        # get target position
        (t_pos, t_vel) = data.target.update(dt)
        # get drone internal data
        internal = get_drone_kf_data()

        # compute measured R and Sigma
        r = gu.compute_r(drone_pos, t_pos)
        sigma = gu.compute_sigma(drone_pos, t_pos)

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
        r_dot_kf = gu.compute_r_dot(internal[0:2], internal[3:5], t_pos, t_est_vel, r)
        sigma_dot_kf = gu.compute_sigma_dot(internal[0:2], internal[3:5], t_pos,
                                            t_est_vel, r)

        mutex.acquire(blocking=True)
        # scrivere le variabili
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
        guidance_variable[access_data['time']] = new_time
        guidance_variable[17] = d_est_acc[0]
        guidance_variable[18] = d_est_acc[1]

        data.logger.append(guidance_variable)
        mutex.release()
    # print(f'siamo nella fase {phase} al tempo {new_time}')

class DataCore:
    def __init__(self, vicon_freq, target, beta, drone_vel, scf):
        self.vicon_freq = vicon_freq
        # mutex per accedere alle variabili globali contententi il filtro del drone ed il flag si sincronizzazione
        self.scf = scf
        self.counter = 0
        self.dt = 0.01
        self.old_frame_number = 0
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(0,
                                              self.dt, update_data_core, self))
        self.drone_vel = drone_vel
        # se lo drone_check è a zero deve girare solo le correzioni e controlla quando avviene il fronte in salita
        # se lo drone_check è 1 inizializza i filtri e poi passa allo stato due
        # se lo drone_check è 2 fa l'update delle sue variabili
        self.drone_check = 0
        self.target = target
        self.target_ff = FadingFilter(dimensions=2, order=3, beta=beta[0])
        self.drone_ff = FadingFilter(dimensions=2, order=3, beta=beta[1])
        self.logger = crazy.AsyncMatlabPrint(flag=6, num_data=19)

    #  self.logger =  definire ill logger per la scrittura asincrona

    # (est_t_pos[0], est_t_pos[1],est_pur_pos[0], est_pur_pos[1], && stessa cosa con le velocità, (4)
    # r, sigma,
    # r_dot,sigma_dot, (calcollate con V dai FF)
    # r_dot,sigma_dot, (calcollate con V dai FF e Kalman fillter)
    # matlab_time,
    # est_t_acc[0], est_t_acc[1],est_p_acc[0], est_p_acc[1])
    def start(self):
        self.update_thread.daemon = True
        self.update_thread.start()

    def stop(self):
        self.update_thread.join()


