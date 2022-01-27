import logging
import math
import threading
import time
import numpy as np
from own_module import crazyfun as crazy
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s

def update_virtual_target(obj_target,dt=0.05):
    # integrate velociticy and acceleration with forward euler
    obj_target.mutex.acquire(True)
    obj_target.time_line = np.append(obj_target.time_line,np.array(time.time()))
    if len(obj_target.time_line) < 2:
        deltat = dt
    else:
        deltat = obj_target.time_line[-1] - obj_target.time_line[-2]
    if np.linalg.norm(obj_target.v,2) != 0:

        obj_target.p += obj_target.v * deltat + obj_target.omega_vers_hat.dot(obj_target.v/np.linalg.norm(obj_target.v,2))*obj_target.a* deltat * deltat /2
        obj_target.v += obj_target.omega_vers_hat.dot(obj_target.v/np.linalg.norm(obj_target.v,2))*obj_target.a* deltat

    obj_target.list_pos = np.concatenate((obj_target.list_pos,obj_target.p.reshape((1,3))),axis=0)
    obj_target.mutex.release()
    if not crazy.run:
        print('Extra Run')
def  update_wand_target(obj_target):
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

        # Get current drone position and orientation in Vicon
    sc_v.wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters
    obj_target.mutex.acquire(True)
    obj_target.time_line = np.append(obj_target.time_line,
                                     np.array(time.time()))
    obj_target.p = np.array([float(sc_v.wand_pos[0] / 1000),
                     float(sc_v.wand_pos[1] / 1000),
                     float(sc_v.wand_pos[2] / 1000)])
    obj_target.mutex.release()
    if not crazy.run:
        print('Extra Run')

class target():
    def __init__(self, initial_position=np.array([0.0, 0.0, 0.5]),
                 initial_velocity=np.array([0.0, 0.0, 0.0]),
                 initial_acceleration_module = 0,
                 dt=0.05,use_wand_target=False):#np.array([0.0, 0.0, 0.0])):
        self.p = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration_module
        self.wand = use_wand_target
        self.list_pos = initial_position.reshape((1,3))
        self.time_line = np.array([0.0])
        self.omega_vers_hat = np.array([[0,-1,0],[1,0,0],[0,0,0]])
        if not self.wand:
            self.update_thread = threading.Thread(target= crazy.repeat_fun,args=(dt,update_virtual_target, self, dt) )
        else:
            self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                                  args=(
                                                  dt, update_wand_target,
                                                  self))
        self.mutex = threading.Semaphore(1)

    def initial_wand(self, in_p):
        if self.wand:
            self.p = in_p

    def get_target(self):
        self.mutex.acquire(True)
        if len(np.argwhere(self.v == 0)) < 3:
            acc = self.omega_vers_hat.dot(
                self.v / np.linalg.norm(self.v, 2)) * self.a
        else:
            acc = np.zeros(3)
        if self.time_line[0] == 0.0:
            ret = (self.p, self.v, acc, 0.0)
        else:
            ret = (self.p, self.v, acc, self.time_line[-1])
        self.mutex.release()
        return ret

    def start(self):
        print('Starting Target Thread')
        self.time_line[0]=time.time()
        crazy.run = True
        self.update_thread.start()

    def stop(self):
        print('Stopping Target Thread')
        crazy.run = False
        self.update_thread.join()
  


