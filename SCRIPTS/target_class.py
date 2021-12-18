import logging
import math
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
#from own_module import crazyfun as crazy, script_setup as sc_s, \
    #script_variables as sc_v
#from vicon_dssdk import ViconDataStream
run=True
def repeat_fun(period, func, *args):
    """
    Uses an internal generator function to run another function
    at a set interval of time.

    :param period: Period at which repeat the execution of func.
    :type period: integer
    :param func: Function to execute.
    :type func: callable
    :param args: Arguments to pass to func.
    :type args: any
    :return: None.
    :rtype: None
    """


    def time_tick():
        """
        Generator function that scan the passing of the desired period of time.

        :return: Yields the passed time.
        :rtype: iterator
        """
        t = time.time()
        while True:
            t += period
            yield max(t - time.time(), 0)

    # Initiates the generator function
    tick = time_tick()

    # Uses a global flag to stop the execution
    while run:
        time.sleep(next(tick))
        func(*args)



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
    if not run:
        print('esecuzioni extra non dovute')


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
        self.update_thread = threading.Thread(target= repeat_fun,args=(dt,update_virtual_target, self, dt) )
        self.mutex = threading.Semaphore(1)
    def get_target(self):
        self.mutex.acquire(True)
        ret = (self.p,self.v)
        self.mutex.release()
        return ret
    def start(self):
        global run
        print('avvio il thread')
        run=True
        #self.update_thread.daemon=True
        self.update_thread.start()
    def stop(self):
        global run
        print('fermo il thread')
        run = False
        self.update_thread.join()
        #crazy.run=True

        #plt.figure(1)
        #plt.plot(self.list_pos[0,0],self.list_pos[0,1],'o')
        #plt.plot(self.list_pos[:,0],self.list_pos[:,1],'-')
        #plt.plot(self.list_pos[-1,0],self.list_pos[-1,1],'->')
        #plt.show()
        """
        Logs the Target position to a file.

        :return: None.
        :rtype: None
        """
#a = target(initial_velocity=np.array([1.0,0.0,0.0]),initial_acceleration_module=1.0,dt= 0.001)
#a.start()

#time.sleep(6.28)
#a.stop()
#print('Dovrei avre finito ')

    #     # Get a new frame from Vicon
    #     try:
    #         sc_s.vicon.GetFrame()
    #     except ViconDataStream.DataStreamException as exc:
    #         logging.error("Error while getting a frame in the core! "
    #                       "--> %s", str(exc))
    #
    #     # Get current target position and orientation in Vicon
    #     self.target_pos = sc_s.vicon. \
    #         GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
    #
    #     # Converts in meters
    #     self.target_pos = (float(self.target_pos[0] / 1000),
    #                        float(self.target_pos[1] / 1000),
    #                        float(self.target_pos[2] / 1000))
    #
    #     target_pos = self.target_pos
    #
    #     logging.debug("Acquired Target position: %s", str(self.target_pos))
    #
    #     # Log to file
    #     # target_matlab.write(target_pos[0],
    #     #                         target_pos[1],
    #     #                         target_pos[2])

