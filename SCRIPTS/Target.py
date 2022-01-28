import logging
import threading
import time
import numpy as np
from own_module import crazyfun as crazy
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s


def update_virtual_target(target, dt=0.05):
    # integrate velocity and acceleration with forward euler
    target.mutex.acquire(True)
    target.time_line = np.append(target.time_line, np.array(time.time()))
    if len(target.time_line) < 2:
        deltat = dt
    else:
        deltat = target.time_line[-1] - target.time_line[-2]
    if np.linalg.norm(target.vel,  2) != 0:
        target.pos += target.vel * deltat + target.omega_vers_hat.dot(target.vel/np.linalg.norm(target.vel, 2))*target.acc* deltat * deltat /2
        target.vel += target.omega_vers_hat.dot(target.vel/np.linalg.norm(target.vel, 2))*target.acc*deltat

    target.mutex.release()
    if not crazy.run:
        print('Extra Run')


def update_wand_target(target):
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

    # Get current drone position and orientation in Vicon
    sc_v.wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters
    target.mutex.acquire(True)
    target.time_line = np.append(target.time_line, np.array(time.time()))
    target.pos = np.array([float(sc_v.wand_pos[0] / 1000),
                           float(sc_v.wand_pos[1] / 1000),
                           float(sc_v.wand_pos[2] / 1000)])
    target.mutex.release()
    if not crazy.run:
        print('Extra Run')


# commentare i parametri di inizializzazione => fondamentale il dt
class Target:
    def __init__(self, initial_pos,
                 initial_vel=np.array([0.0, 0.0, 0.0]),
                 initial_acc_module=0,
                 dt=0.05, use_wand_target=False):
        self.pos = initial_pos
        self.vel = initial_vel
        self.acc = initial_acc_module
        self.wand = use_wand_target
        self.time_line = np.array([0.0])
        self.omega_vers_hat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
        if not self.wand:
            self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                                  args=(dt, update_virtual_target, self, dt))
        else:
            self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                                  args=(dt, update_wand_target, self))
        self.mutex = threading.Semaphore(1)

    def init_wand(self, init_pos):
        if self.wand:
            self.pos = init_pos

    def get_state(self):
        self.mutex.acquire(True)
        if len(np.argwhere(self.vel == 0)) < 3:
            acc = self.omega_vers_hat.dot(
                self.vel / np.linalg.norm(self.vel, 2)) * self.acc
        else:
            acc = np.zeros(3)
        if self.time_line[0] == 0.0:
            ret = (self.pos, self.vel, acc, 0.0)
        else:
            ret = (self.pos, self.vel, acc, self.time_line[-1])
        self.mutex.release()
        return ret

    def start(self):
        print('Starting Target Thread')
        self.time_line[0] = time.time()
        crazy.run = True
        self.update_thread.start()

    def stop(self):
        print('Stopping Target Thread')
        crazy.run = False
        self.update_thread.join()
        