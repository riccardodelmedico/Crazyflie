import logging
import math
import threading
import time
import numpy as np
from own_module import crazyfun as crazy


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


class target():
    def __init__(self, initial_position=np.array([0.0, 0.0, 0.5]),
                 initial_velocity=np.array([0.0, 0.0, 0.0]),
                 initial_acceleration_module = 0,
                 dt=0.05):#np.array([0.0, 0.0, 0.0])):
        self.p = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration_module
        self.list_pos = initial_position.reshape((1,3))
        self.time_line = np.array([])
        self.omega_vers_hat = np.array([[0,-1,0],[1,0,0],[0,0,0]])
        self.update_thread = threading.Thread(target= crazy.repeat_fun,args=(dt,update_virtual_target, self, dt) )
        self.mutex = threading.Semaphore(1)

    def get_target(self):
        self.mutex.acquire(True)
        ret = (self.p, self.v)
        self.mutex.release()
        return ret

    def start(self):
        print('Starting Target Thread')
        crazy.run = True
        self.update_thread.start()

    def stop(self):
        print('Stopping Target Thread')
        crazy.run = False
        self.update_thread.join()


